/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/FileGraph.h"
#include "common/NetworkFunction.h"
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <tuple>
#include <yaml-cpp/yaml.h>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

namespace {

[[noreturn]] void reject_graph(const std::string& message) noexcept {
    std::cerr << "[Error] (network/analytical/congestion_aware) "
              << message << std::endl;
    std::abort();
}

LinkClass parse_link_class(const YAML::Node& edge) {
    if (!edge["class"]) {
        return LinkClass::Generic;
    }
    const auto value = edge["class"].as<std::string>();
    if (value == "direct") {
        return LinkClass::BaseMesh;
    }
    if (value == "switch") {
        return LinkClass::SwitchUplink;
    }
    if (value == "optical") {
        return LinkClass::SwitchUplink;
    }
    if (value == "generic") {
        return LinkClass::Generic;
    }
    reject_graph("unknown file-graph link class: " + value);
}

int graph_devices(const std::string& path) {
    try {
        return YAML::LoadFile(path)["devices"].as<int>();
    } catch (const YAML::Exception& error) {
        reject_graph(std::string("cannot read file-graph header: ") +
                     error.what());
    }
}

}  // namespace

FileGraph::FileGraph(const int npus_count, const Bandwidth bandwidth,
                     const Latency latency,
                     const std::string& graph_path) noexcept
    : BasicTopology(npus_count, graph_devices(graph_path), bandwidth, latency),
      adjacency(devices_count) {
    if (graph_path.empty()) {
        reject_graph("FileGraph requires routing_plan");
    }
    try {
        const auto root = YAML::LoadFile(graph_path);
        if (!root["format"] ||
            root["format"].as<std::string>() != "panel-physical-graph" ||
            !root["version"] || root["version"].as<int>() != 1 ||
            !root["endpoints"] || root["endpoints"].as<int>() != npus_count ||
            !root["devices"] || root["devices"].as<int>() != devices_count ||
            !root["edges"] || !root["edges"].IsSequence()) {
            reject_graph("invalid file-graph header");
        }
        const auto directed = root["directed"] && root["directed"].as<bool>();
        auto installed = std::set<std::pair<DeviceId, DeviceId>>();
        for (const auto& edge : root["edges"]) {
            if (!edge.IsMap() || !edge["source"] || !edge["destination"]) {
                reject_graph("file-graph edge must name source and destination");
            }
            const auto source = edge["source"].as<DeviceId>();
            const auto destination = edge["destination"].as<DeviceId>();
            const auto edge_bandwidth = edge["bandwidth"]
                                            ? edge["bandwidth"].as<Bandwidth>()
                                            : bandwidth;
            const auto edge_latency = edge["latency"]
                                          ? edge["latency"].as<Latency>()
                                          : latency;
            if (source < 0 || source >= devices_count || destination < 0 ||
                destination >= devices_count || source == destination ||
                edge_bandwidth <= 0 || edge_latency < 0) {
                reject_graph("invalid file-graph edge values");
            }
            const auto key = std::make_pair(source, destination);
            const auto reverse = std::make_pair(destination, source);
            if (!installed.insert(key).second ||
                (!directed && !installed.insert(reverse).second)) {
                reject_graph("duplicate file-graph edge");
            }
            const auto ports = connect(source, destination, edge_bandwidth,
                                       edge_latency, !directed,
                                       parse_link_class(edge));
            adjacency[source].push_back({destination, ports.first});
            if (!directed) {
                adjacency[destination].push_back({source, ports.second});
            }
        }
        for (auto& arcs : adjacency) {
            std::sort(arcs.begin(), arcs.end(), [](const Arc& first,
                                                   const Arc& second) {
                return std::tie(first.destination, first.source_port) <
                       std::tie(second.destination, second.source_port);
            });
        }
    } catch (const YAML::Exception& error) {
        reject_graph(std::string("cannot load file graph: ") + error.what());
    }
    basic_topology_type = TopologyBuildingBlock::FileGraph;
}

double FileGraph::arc_cost(const DeviceId source, const Arc& arc,
                           const ChunkSize bytes) const noexcept {
    const auto link_bandwidth =
        devices[source]->get_link_bandwidth(arc.source_port);
    return devices[source]->get_link_latency(arc.source_port) +
           (static_cast<double>(
                devices[source]->get_outstanding_bytes(arc.source_port)) +
            static_cast<double>(bytes)) /
               bw_GBps_to_Bpns(link_bandwidth);
}

Route FileGraph::route(const DeviceId src, const DeviceId dest) const noexcept {
    return route(src, dest, 1);
}

Route FileGraph::route(const DeviceId src, const DeviceId dest,
                       const ChunkSize chunk_size) const noexcept {
    assert(src >= 0 && src < npus_count && dest >= 0 && dest < npus_count);
    assert(chunk_size > 0);
    if (src == dest) {
        return Route({devices[src]});
    }

    const auto infinity = std::numeric_limits<double>::infinity();
    auto distance = std::vector<double>(devices_count, infinity);
    auto predecessor = std::vector<DeviceId>(devices_count, -1);
    auto predecessor_arc = std::vector<int>(devices_count, -1);
    using QueueEntry = std::pair<double, DeviceId>;
    auto queue = std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                                     std::greater<QueueEntry>>();
    distance[src] = 0.0;
    queue.emplace(0.0, src);
    while (!queue.empty()) {
        const auto [current_distance, current] = queue.top();
        queue.pop();
        if (current_distance != distance[current]) {
            continue;
        }
        if (current == dest) {
            break;
        }
        for (auto index = 0;
             index < static_cast<int>(adjacency[current].size()); ++index) {
            const auto& arc = adjacency[current][index];
            const auto candidate = current_distance +
                                   arc_cost(current, arc, chunk_size);
            if (candidate < distance[arc.destination]) {
                distance[arc.destination] = candidate;
                predecessor[arc.destination] = current;
                predecessor_arc[arc.destination] = index;
                queue.emplace(candidate, arc.destination);
            }
        }
    }
    if (predecessor[dest] < 0) {
        reject_graph("file graph cannot route between endpoints");
    }

    auto reversed = std::vector<std::pair<DeviceId, int>>();
    for (auto current = dest; current != src; current = predecessor[current]) {
        reversed.emplace_back(predecessor[current], predecessor_arc[current]);
    }
    std::reverse(reversed.begin(), reversed.end());
    auto result = Route();
    auto current = src;
    for (const auto& [source, index] : reversed) {
        assert(source == current);
        const auto& arc = adjacency[source][index];
        result.emplace_back(devices[source], arc.source_port);
        current = arc.destination;
    }
    result.emplace_back(devices[dest]);
    return result;
}
