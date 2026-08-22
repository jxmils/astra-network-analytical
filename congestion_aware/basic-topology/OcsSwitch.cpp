/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/OcsSwitch.h"
#include "common/NetworkFunction.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <ostream>
#include <set>
#include <yaml-cpp/yaml.h>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

namespace {

[[noreturn]] void reject_ocs(const std::string& message) noexcept {
    std::cerr << "[Error] (network/analytical/congestion_aware) OCS: "
              << message << std::endl;
    std::abort();
}

EventTime positive_event_delay(const double delay) noexcept {
    return std::max<EventTime>(1, static_cast<EventTime>(std::ceil(delay)));
}

}  // namespace

OcsSwitch::OcsSwitch(const int npus_count, const Bandwidth bandwidth,
                     const Latency latency, const std::string& plan_path,
                     const int expected_planes, const bool base_torus) noexcept
    : BasicTopology(npus_count, npus_count + expected_planes, bandwidth, latency),
      planes(0), expected_planes(expected_planes), base_torus(base_torus),
      width(static_cast<int>(std::lround(std::sqrt(static_cast<double>(npus_count))))),
      plan_bandwidth(0), propagation_ns(0), reconfiguration_ns(0),
      initial_reconfiguration(false), current_round(0), reconfiguring(false),
      completed_rounds(0), reconfiguration_count(0), scheduled_bytes(0),
      transmitted_bytes(0), planned_assignments(0), consumed_assignments(0),
      reconfiguration_time(0) {
    basic_topology_type = base_torus ? TopologyBuildingBlock::TorusOcsStatic2D
                                     : TopologyBuildingBlock::OcsSwitch6;
    load_plan(plan_path);
    validate_plan();

    if (base_torus) {
        dims_count = 2;
        npus_count_per_dim = {width, width};
        bandwidth_per_dim = {bandwidth, bandwidth};
        build_base_torus();
    }

    to_switch_ports.assign(planes, std::vector<LinkId>(npus_count));
    from_switch_ports.assign(planes, std::vector<LinkId>(npus_count));
    for (auto plane = 0; plane < planes; ++plane) {
        const auto switch_id = npus_count + plane;
        for (auto npu = 0; npu < npus_count; ++npu) {
            const auto ports = connect(npu, switch_id, plan_bandwidth,
                                       propagation_ns / 2.0, true,
                                       LinkClass::SwitchUplink);
            to_switch_ports[plane][npu] = ports.first;
            from_switch_ports[plane][npu] = ports.second;
            physical_metrics[{npu, ports.first}] = {
                npu, switch_id, ports.first, LinkClass::SwitchUplink,
                0, 0, 0, 0, 0};
            physical_metrics[{switch_id, ports.second}] = {
                switch_id, npu, ports.second, LinkClass::SwitchUplink,
                0, 0, 0, 0, 0};
        }
    }
}

void OcsSwitch::load_plan(const std::string& path) noexcept {
    if (path.empty()) {
        reject_ocs("topology requires an ocs_plan file");
    }
    try {
        const auto root = YAML::LoadFile(path);
        if (root["format"].as<std::string>() != "panel-ocs-plan" ||
            root["version"].as<int>() != 2) {
            reject_ocs("unsupported plan format or version");
        }
        if (root["endpoints"].as<int>() != npus_count) {
            reject_ocs("plan endpoint count does not match topology");
        }
        planes = root["planes"].as<int>();
        plan_bandwidth = root["link_bandwidth_GBps"].as<double>();
        propagation_ns = root["propagation_ns"].as<double>();
        reconfiguration_ns = root["reconfiguration_ns"].as<double>();
        initial_reconfiguration = root["initial_reconfiguration"].as<bool>();
        if (!root["assignments"] || !root["assignments"].IsSequence()) {
            reject_ocs("plan has no route assignments");
        }
        for (const auto& assignment : root["assignments"]) {
            const auto source = assignment["source"].as<int>();
            const auto destination = assignment["destination"].as<int>();
            const auto bytes = assignment["bytes"].as<uint64_t>();
            const auto stream = assignment["stream"].as<int>();
            const auto route = assignment["route"].as<std::string>();
            if (source < 0 || source >= npus_count || destination < 0 ||
                destination >= npus_count || source == destination || bytes == 0 ||
                stream < 0 || (route != "DIRECT" && route != "OCS")) {
                reject_ocs("invalid route assignment");
            }
            route_assignments[{source, destination, bytes, stream}].push_back(
                route == "OCS");
            ++planned_assignments;
        }
        for (const auto& round_node : root["rounds"]) {
            auto circuit_round = Round{round_node["index"].as<int>(), {}};
            for (const auto& configuration_node : round_node["configurations"]) {
                auto configuration = Configuration{
                    configuration_node["plane"].as<int>(), {}};
                for (const auto& circuit_node : configuration_node["circuits"]) {
                    const auto source = circuit_node["source"].as<int>();
                    const auto destination = circuit_node["destination"].as<int>();
                    const auto bytes = circuit_node["bytes"].as<uint64_t>();
                    configuration.circuits.push_back(
                        Circuit{{source, destination}, bytes, bytes, false});
                    scheduled_bytes += bytes;
                }
                circuit_round.configurations.push_back(std::move(configuration));
            }
            rounds.push_back(std::move(circuit_round));
        }
    } catch (const YAML::Exception& error) {
        reject_ocs(std::string("could not parse plan: ") + error.what());
    }
}

void OcsSwitch::validate_plan() const noexcept {
    if (planes != expected_planes) {
        reject_ocs("plan plane count does not match topology");
    }
    if (!base_torus && std::abs(plan_bandwidth - bandwidth) > 1e-9) {
        reject_ocs("plan link speed does not match network bandwidth");
    }
    if (!base_torus && std::abs(propagation_ns - 2.0 * latency) > 1e-9) {
        reject_ocs("plan propagation must equal two physical-link latencies");
    }
    if (base_torus && width * width != npus_count) {
        reject_ocs("torus Hybrid requires a square endpoint count");
    }
    if (reconfiguration_ns < 0) {
        reject_ocs("reconfiguration latency must be nonnegative");
    }
    for (std::size_t index = 0; index < rounds.size(); ++index) {
        const auto& circuit_round = rounds[index];
        if (circuit_round.index != static_cast<int>(index)) {
            reject_ocs("plan round indices are not contiguous");
        }
        auto used_planes = std::set<int>();
        for (const auto& configuration : circuit_round.configurations) {
            if (configuration.plane < 0 || configuration.plane >= planes ||
                !used_planes.insert(configuration.plane).second) {
                reject_ocs("invalid or repeated plane in a round");
            }
            auto sources = std::set<int>();
            auto destinations = std::set<int>();
            for (const auto& circuit : configuration.circuits) {
                const auto [source, destination] = circuit.pair;
                if (source < 0 || source >= npus_count || destination < 0 ||
                    destination >= npus_count || source == destination ||
                    circuit.bytes == 0 || !sources.insert(source).second ||
                    !destinations.insert(destination).second) {
                    reject_ocs("invalid directional matching");
                }
            }
        }
    }
}

Route OcsSwitch::route(const DeviceId src, const DeviceId dest) const noexcept {
    return route(src, dest, 1);
}

Route OcsSwitch::route(const DeviceId src, const DeviceId dest,
                       const ChunkSize chunk_size) const noexcept {
    return route(src, dest, chunk_size, 0);
}

Route OcsSwitch::route(const DeviceId src, const DeviceId dest,
                       const ChunkSize chunk_size, const int stream) const noexcept {
    assert(src >= 0 && src < npus_count && dest >= 0 && dest < npus_count);
    assert(chunk_size > 0);
    if (src == dest) {
        return Route({devices[src]});
    }
    const auto key = AssignmentKey{src, dest, chunk_size, stream};
    const auto found = route_assignments.find(key);
    if (found == route_assignments.end() || found->second.empty()) {
        reject_ocs("plan has no remaining route assignment for " +
                   std::to_string(src) + " -> " + std::to_string(dest) +
                   " size " + std::to_string(chunk_size) + " stream " +
                   std::to_string(stream));
    }
    const auto use_ocs = found->second.front();
    found->second.pop_front();
    ++consumed_assignments;
    if (!use_ocs) {
        if (!base_torus) {
            reject_ocs("direct assignment requires a persistent base fabric");
        }
        return direct_route(src, dest);
    }
    auto path = Route();
    path.emplace_back(devices[src], to_switch_ports[0][src]);
    path.emplace_back(devices[npus_count], from_switch_ports[0][dest]);
    path.emplace_back(devices[dest]);
    return path;
}

void OcsSwitch::send(std::unique_ptr<Chunk> chunk) noexcept {
    assert(chunk != nullptr);
    const auto& path = chunk->get_route();
    if (path.size() == 1) {
        chunk->invoke_callback();
        return;
    }
    const auto first_link = path.front().device->get_link_class(
        path.front().outgoing_link);
    if (first_link == LinkClass::BaseMesh) {
        Topology::send(std::move(chunk));
        return;
    }
    record_route(path, chunk->get_size());
    const auto pair = Pair{path.front().device->get_id(), path.back().device->get_id()};
    chunk->mark_link_queued(event_queue->get_current_time());
    pending[pair].push_back(std::move(chunk));
    if (current_round == 0 && completed_rounds == 0 && initial_reconfiguration &&
        !reconfiguring) {
        start_initial_round();
        return;
    }
    try_start_transmissions();
}

void OcsSwitch::start_initial_round() noexcept {
    ++reconfiguration_count;
    if (reconfiguration_ns == 0) {
        try_start_transmissions();
        return;
    }
    reconfiguring = true;
    const auto delay = positive_event_delay(reconfiguration_ns);
    reconfiguration_time += delay;
    event_queue->schedule_event(event_queue->get_current_time() + delay,
                                reconfiguration_callback, this);
}

OcsSwitch::Circuit* OcsSwitch::find_circuit(const int plane,
                                            const Pair& pair) noexcept {
    if (current_round >= static_cast<int>(rounds.size())) {
        return nullptr;
    }
    for (auto& configuration : rounds[current_round].configurations) {
        if (configuration.plane != plane) {
            continue;
        }
        for (auto& circuit : configuration.circuits) {
            if (circuit.pair == pair) {
                return &circuit;
            }
        }
    }
    return nullptr;
}

void OcsSwitch::try_start_transmissions() noexcept {
    if (reconfiguring || current_round >= static_cast<int>(rounds.size())) {
        return;
    }
    auto progress = true;
    while (progress) {
        progress = false;
        for (auto& configuration : rounds[current_round].configurations) {
            for (auto& circuit : configuration.circuits) {
                auto found = pending.find(circuit.pair);
                if (circuit.busy || circuit.remaining == 0 || found == pending.end() ||
                    found->second.empty()) {
                    continue;
                }
                auto& chunk = found->second.front();
                if (chunk->get_size() > circuit.remaining) {
                    reject_ocs("chunk exceeds the remaining circuit byte quota");
                }
                auto selected = std::move(chunk);
                found->second.pop_front();
                start_transmission(configuration.plane, circuit, std::move(selected));
                progress = true;
            }
        }
    }
}

void OcsSwitch::start_transmission(const int plane, Circuit& circuit,
                                   std::unique_ptr<Chunk> chunk) noexcept {
    circuit.busy = true;
    const auto bytes = chunk->get_size();
    const auto [source, destination] = circuit.pair;
    const auto switch_id = npus_count + plane;
    const auto source_port = to_switch_ports[plane][source];
    const auto destination_port = from_switch_ports[plane][destination];
    const auto serialization = positive_event_delay(
        static_cast<double>(bytes) / bw_GBps_to_Bpns(plan_bandwidth));
    const auto wait = event_queue->get_current_time() - chunk->get_link_queued_time();
    for (auto key : {std::make_pair(source, source_port),
                     std::make_pair(switch_id, destination_port)}) {
        auto& metric = physical_metrics.at(key);
        metric.bytes += bytes;
        ++metric.messages;
        metric.peak_outstanding_bytes = std::max(metric.peak_outstanding_bytes,
                                                 static_cast<uint64_t>(bytes));
        metric.busy_time += serialization;
        metric.queue_wait_time += wait;
    }
    auto* transmission = new Transmission{
        this, std::move(chunk), plane, circuit.pair, bytes};
    event_queue->schedule_event(event_queue->get_current_time() + serialization,
                                serialization_callback, transmission);
}

void OcsSwitch::serialization_callback(void* argument) noexcept {
    auto* transmission = static_cast<Transmission*>(argument);
    transmission->topology->finish_serialization(transmission);
}

void OcsSwitch::arrival_callback(void* argument) noexcept {
    auto transmission = std::unique_ptr<Transmission>(
        static_cast<Transmission*>(argument));
    transmission->topology->finish_arrival(transmission.get());
}

void OcsSwitch::reconfiguration_callback(void* argument) noexcept {
    auto* topology = static_cast<OcsSwitch*>(argument);
    topology->reconfiguring = false;
    topology->try_start_transmissions();
}

void OcsSwitch::finish_serialization(Transmission* transmission) noexcept {
    auto* circuit = find_circuit(transmission->plane, transmission->pair);
    if (circuit == nullptr || !circuit->busy || circuit->remaining < transmission->bytes) {
        reject_ocs("serialization completed outside its installed circuit");
    }
    circuit->remaining -= transmission->bytes;
    circuit->busy = false;
    transmitted_bytes += transmission->bytes;
    const auto propagation = positive_event_delay(propagation_ns);
    event_queue->schedule_event(event_queue->get_current_time() + propagation,
                                arrival_callback, transmission);
    try_start_transmissions();
    if (round_complete()) {
        advance_round();
    }
}

void OcsSwitch::finish_arrival(Transmission* transmission) noexcept {
    transmission->chunk->invoke_callback();
}

bool OcsSwitch::round_complete() const noexcept {
    if (current_round >= static_cast<int>(rounds.size())) {
        return false;
    }
    for (const auto& configuration : rounds[current_round].configurations) {
        for (const auto& circuit : configuration.circuits) {
            if (circuit.remaining != 0 || circuit.busy) {
                return false;
            }
        }
    }
    return true;
}

bool OcsSwitch::configuration_changed(const Round& first,
                                      const Round& second) const noexcept {
    for (auto plane = 0; plane < planes; ++plane) {
        auto support = [plane](const Round& circuit_round) {
            auto result = std::vector<Pair>();
            for (const auto& configuration : circuit_round.configurations) {
                if (configuration.plane != plane) {
                    continue;
                }
                for (const auto& circuit : configuration.circuits) {
                    result.push_back(circuit.pair);
                }
            }
            std::sort(result.begin(), result.end());
            return result;
        };
        if (support(first) != support(second)) {
            return true;
        }
    }
    return false;
}

void OcsSwitch::advance_round() noexcept {
    const auto previous = current_round;
    ++current_round;
    ++completed_rounds;
    if (current_round >= static_cast<int>(rounds.size())) {
        for (const auto& [pair, queue] : pending) {
            if (!queue.empty()) {
                reject_ocs("plan completed with unscheduled pending traffic");
            }
        }
        return;
    }
    if (configuration_changed(rounds[previous], rounds[current_round])) {
        ++reconfiguration_count;
        if (reconfiguration_ns == 0) {
            try_start_transmissions();
            return;
        }
        reconfiguring = true;
        const auto delay = positive_event_delay(reconfiguration_ns);
        reconfiguration_time += delay;
        event_queue->schedule_event(event_queue->get_current_time() + delay,
                                    reconfiguration_callback, this);
    } else {
        try_start_transmissions();
    }
}

int OcsSwitch::get_plane_count() const noexcept { return planes; }
int OcsSwitch::get_round_count() const noexcept { return static_cast<int>(rounds.size()); }
int OcsSwitch::get_completed_rounds() const noexcept { return completed_rounds; }
int OcsSwitch::get_reconfiguration_count() const noexcept { return reconfiguration_count; }
uint64_t OcsSwitch::get_scheduled_bytes() const noexcept { return scheduled_bytes; }
uint64_t OcsSwitch::get_transmitted_bytes() const noexcept { return transmitted_bytes; }
EventTime OcsSwitch::get_reconfiguration_time() const noexcept { return reconfiguration_time; }

std::vector<LinkMetrics> OcsSwitch::get_link_metrics() const noexcept {
    auto result = Topology::get_link_metrics();
    for (auto& metric : result) {
        const auto found = physical_metrics.find({metric.source, metric.port});
        if (found != physical_metrics.end()) {
            metric = found->second;
        }
    }
    return result;
}

void OcsSwitch::print_link_metrics(std::ostream& output) const {
    Topology::print_link_metrics(output);
    output << "OCS_SUMMARY planes=" << planes
           << " rounds=" << rounds.size()
           << " completed_rounds=" << completed_rounds
           << " reconfigurations=" << reconfiguration_count
           << " reconfiguration_ns=" << reconfiguration_time
           << " scheduled_bytes=" << scheduled_bytes
           << " transmitted_bytes=" << transmitted_bytes
           << " assignments=" << planned_assignments
           << " consumed_assignments=" << consumed_assignments << '\n';
}

void OcsSwitch::build_base_torus() noexcept {
    auto remember = [this](const DeviceId first, const DeviceId second) {
        const auto ports = connect(first, second, bandwidth, latency, true,
                                   LinkClass::BaseMesh);
        base_ports[{first, second}] = ports.first;
        base_ports[{second, first}] = ports.second;
    };
    for (auto y = 0; y < width; ++y) {
        for (auto x = 0; x < width; ++x) {
            const auto node = y * width + x;
            if (x + 1 < width) {
                remember(node, node + 1);
            } else {
                remember(node, y * width);
            }
            if (y + 1 < width) {
                remember(node, node + width);
            } else {
                remember(node, x);
            }
        }
    }
}

int OcsSwitch::step_towards(const int current, const int target, const int extent,
                            const bool tie_backward) const noexcept {
    const auto forward = (target - current + extent) % extent;
    const auto backward = extent - forward;
    if (forward < backward) {
        return (current + 1) % extent;
    }
    if (backward < forward) {
        return (current - 1 + extent) % extent;
    }
    return tie_backward ? (current - 1 + extent) % extent
                        : (current + 1) % extent;
}

Route OcsSwitch::direct_route(const DeviceId src, const DeviceId dest) const noexcept {
    auto path = Route();
    auto current = src;
    auto x = src % width;
    auto y = src / width;
    const auto destination_x = dest % width;
    const auto destination_y = dest / width;
    while (x != destination_x) {
        const auto next_x = step_towards(x, destination_x, width, (x & 1) != 0);
        const auto next = y * width + next_x;
        path.emplace_back(devices[current], base_ports.at({current, next}));
        current = next;
        x = next_x;
    }
    while (y != destination_y) {
        const auto next_y = step_towards(y, destination_y, width, (y & 1) != 0);
        const auto next = next_y * width + x;
        path.emplace_back(devices[current], base_ports.at({current, next}));
        current = next;
        y = next_y;
    }
    path.emplace_back(devices[dest]);
    return path;
}
