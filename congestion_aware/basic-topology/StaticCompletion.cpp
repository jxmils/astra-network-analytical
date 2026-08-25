/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/StaticCompletion.h"
#include "common/NetworkFunction.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <utility>
#include <yaml-cpp/yaml.h>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

namespace {

[[noreturn]] void reject_completion(const std::string& message) noexcept {
    std::cerr << "[Error] (network/analytical/congestion_aware) "
              << message << std::endl;
    std::abort();
}

}  // namespace

StaticCompletion::StaticCompletion(
    const int npus_count, const Bandwidth bandwidth, const Latency latency,
    const Bandwidth optical_bandwidth, const Latency optical_leg_latency,
    const std::string& plan_path) noexcept
    : BasicTopology(npus_count, npus_count + 2, bandwidth, latency),
      side(static_cast<int>(std::lround(std::sqrt(npus_count)))),
      planes(2),
      optical_bandwidth(optical_bandwidth),
      optical_leg_latency(optical_leg_latency),
      adjacency(npus_count) {
    if (side * side != npus_count) {
        reject_completion("static completion requires a square endpoint count");
    }
    if (optical_bandwidth <= 0 || optical_leg_latency < 0) {
        reject_completion("static completion has invalid optical link parameters");
    }
    if (plan_path.empty()) {
        reject_completion("static completion requires a routing_plan file");
    }
    basic_topology_type = TopologyBuildingBlock::TorusStaticCompletion2D;
    build_base_torus();
    load_matchings(plan_path);
}

void StaticCompletion::build_base_torus() noexcept {
    const auto add_edge = [this](const DeviceId first, const DeviceId second) {
        const auto ports = connect(first, second, bandwidth, latency, true,
                                   LinkClass::BaseMesh);
        adjacency[first].push_back({second, -1, ports.first, AutomaticLink});
        adjacency[second].push_back({first, -1, ports.second, AutomaticLink});
    };
    for (auto y = 0; y < side; ++y) {
        for (auto x = 0; x < side; ++x) {
            const auto source = y * side + x;
            if (x + 1 < side) {
                add_edge(source, y * side + x + 1);
            } else if (side > 2) {
                add_edge(source, y * side);
            }
            if (y + 1 < side) {
                add_edge(source, (y + 1) * side + x);
            } else if (side > 2) {
                add_edge(source, x);
            }
        }
    }
}

void StaticCompletion::load_matchings(const std::string& path) noexcept {
    try {
        const auto root = YAML::LoadFile(path);
        if (!root["format"] || root["format"].as<std::string>() !=
                                   "panel-static-completion" ||
            !root["version"] || root["version"].as<int>() != 1 ||
            !root["endpoints"] || root["endpoints"].as<int>() != npus_count ||
            !root["matchings"] || !root["matchings"].IsSequence() ||
            static_cast<int>(root["matchings"].size()) != planes) {
            reject_completion("invalid static-completion plan header");
        }

        auto endpoint_ports = std::vector<std::vector<LinkId>>(
            planes, std::vector<LinkId>(npus_count));
        auto switch_ports = std::vector<std::vector<LinkId>>(
            planes, std::vector<LinkId>(npus_count));
        for (auto plane = 0; plane < planes; ++plane) {
            const auto switch_id = npus_count + plane;
            for (auto endpoint = 0; endpoint < npus_count; ++endpoint) {
                const auto ports = connect(
                    endpoint, switch_id, optical_bandwidth,
                    optical_leg_latency, true, LinkClass::SwitchUplink);
                endpoint_ports[plane][endpoint] = ports.first;
                switch_ports[plane][endpoint] = ports.second;
            }
        }

        auto installed = std::set<std::pair<int, int>>();
        for (auto plane = 0; plane < planes; ++plane) {
            const auto matching = root["matchings"][plane];
            if (!matching.IsSequence() ||
                static_cast<int>(matching.size()) != npus_count / 2) {
                reject_completion("each optical plane must contain a perfect matching");
            }
            auto degree = std::vector<int>(npus_count, 0);
            for (const auto& pair : matching) {
                if (!pair.IsSequence() || pair.size() != 2) {
                    reject_completion("invalid static matching pair");
                }
                const auto first = pair[0].as<int>();
                const auto second = pair[1].as<int>();
                if (first < 0 || first >= npus_count || second < 0 ||
                    second >= npus_count || first == second ||
                    ++degree[first] != 1 || ++degree[second] != 1) {
                    reject_completion("static matching is not perfect");
                }
                const auto edge = std::make_pair(std::min(first, second),
                                                 std::max(first, second));
                if (!installed.insert(edge).second) {
                    reject_completion("static completion repeats an optical edge");
                }
                const auto duplicates_base = std::any_of(
                    adjacency[first].begin(), adjacency[first].end(),
                    [second](const Arc& arc) {
                        return arc.plane < 0 && arc.destination == second;
                    });
                if (duplicates_base) {
                    reject_completion("static completion duplicates a torus edge");
                }
                adjacency[first].push_back({
                    second, plane, endpoint_ports[plane][first],
                    switch_ports[plane][second]});
                adjacency[second].push_back({
                    first, plane, endpoint_ports[plane][second],
                    switch_ports[plane][first]});
            }
            if (std::any_of(degree.begin(), degree.end(),
                            [](const int value) { return value != 1; })) {
                reject_completion("static matching omits an endpoint");
            }
        }
    } catch (const YAML::Exception& error) {
        reject_completion(std::string("cannot load static completion: ") + error.what());
    }
}

double StaticCompletion::arc_cost(
    const DeviceId source, const Arc& arc, const ChunkSize bytes) const noexcept {
    const auto first_bandwidth = devices[source]->get_link_bandwidth(arc.source_port);
    auto cost = devices[source]->get_link_latency(arc.source_port) +
                (static_cast<double>(devices[source]->get_outstanding_bytes(
                     arc.source_port)) + static_cast<double>(bytes)) /
                    bw_GBps_to_Bpns(first_bandwidth);
    if (arc.plane < 0) {
        return cost;
    }
    const auto switch_id = npus_count + arc.plane;
    const auto second_bandwidth =
        devices[switch_id]->get_link_bandwidth(arc.switch_port);
    cost += devices[switch_id]->get_link_latency(arc.switch_port) +
            (static_cast<double>(devices[switch_id]->get_outstanding_bytes(
                 arc.switch_port)) + static_cast<double>(bytes)) /
                bw_GBps_to_Bpns(second_bandwidth);
    return cost;
}

Route StaticCompletion::route(const DeviceId src, const DeviceId dest) const noexcept {
    return route(src, dest, 1);
}

Route StaticCompletion::route(
    const DeviceId src, const DeviceId dest, const ChunkSize chunk_size) const noexcept {
    assert(src >= 0 && src < npus_count && dest >= 0 && dest < npus_count);
    assert(chunk_size > 0);
    if (src == dest) {
        return Route({devices[src]});
    }

    const auto infinity = std::numeric_limits<double>::infinity();
    auto distance = std::vector<double>(npus_count, infinity);
    auto predecessor = std::vector<DeviceId>(npus_count, -1);
    auto predecessor_arc = std::vector<int>(npus_count, -1);
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
        for (auto index = 0; index < static_cast<int>(adjacency[current].size());
             ++index) {
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
        reject_completion("static completion graph is disconnected");
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
        if (arc.plane >= 0) {
            result.emplace_back(devices[npus_count + arc.plane], arc.switch_port);
        }
        current = arc.destination;
    }
    result.emplace_back(devices[dest]);
    return result;
}
