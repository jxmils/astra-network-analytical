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
                     const int expected_planes, const bool base_torus,
                     const bool qtp_embedding) noexcept
    : BasicTopology(npus_count, npus_count + expected_planes, bandwidth, latency),
      planes(0), expected_planes(expected_planes), base_torus(base_torus),
      qtp_embedding(qtp_embedding),
      width(static_cast<int>(std::lround(std::sqrt(static_cast<double>(npus_count))))),
      plan_bandwidth(0), propagation_ns(0), reconfiguration_ns(0),
      initial_reconfiguration(false), epoch_started(false), epoch_start(0),
      reconfiguration_count(0), scheduled_bytes(0), transmitted_bytes(0),
      circuit_wait_time(0), max_circuit_wait_time(0), circuit_transmissions(0),
      planned_assignments(0), consumed_assignments(0), causal_dispatches(0),
      max_release_slip(0), max_release_slip_request(-1),
      max_release_slip_planned(0), max_release_slip_actual(0) {
    basic_topology_type = qtp_embedding
                              ? TopologyBuildingBlock::TorusOcsQtp
                              : base_torus
                                    ? TopologyBuildingBlock::TorusOcsStatic2D
                                    : TopologyBuildingBlock::OcsSwitch6;
    load_plan(plan_path);
    validate_plan();

    if (base_torus) {
        if (qtp_embedding) {
            dims_count = 4;
            npus_count_per_dim = {4, 2, 4, 2};
            bandwidth_per_dim = {bandwidth, bandwidth, bandwidth, bandwidth};
            build_qtp_embedding();
        } else {
            dims_count = 2;
            npus_count_per_dim = {width, width};
            bandwidth_per_dim = {bandwidth, bandwidth};
            logical_to_physical.resize(npus_count);
            physical_to_logical.resize(npus_count);
            for (auto endpoint = 0; endpoint < npus_count; ++endpoint) {
                logical_to_physical[endpoint] = endpoint;
                physical_to_logical[endpoint] = endpoint;
            }
        }
        build_base_torus();
    } else if (!plan_dimensions.empty()) {
        dims_count = static_cast<int>(plan_dimensions.size());
        npus_count_per_dim = plan_dimensions;
        bandwidth_per_dim.assign(plan_dimensions.size(), bandwidth);
    }

    active_endpoint_planes.resize(npus_count);
    active_endpoint_tx_planes.resize(npus_count);
    active_endpoint_rx_planes.resize(npus_count);
    active_endpoint_peers.resize(npus_count);
    active_endpoint_tx_peers.resize(npus_count);
    active_endpoint_rx_peers.resize(npus_count);
    max_active_endpoint_ports.assign(npus_count, 0);
    max_active_endpoint_tx_ports.assign(npus_count, 0);
    max_active_endpoint_rx_ports.assign(npus_count, 0);
    max_distinct_endpoint_peers.assign(npus_count, 0);
    max_distinct_endpoint_tx_peers.assign(npus_count, 0);
    max_distinct_endpoint_rx_peers.assign(npus_count, 0);

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
            root["version"].as<int>() != 5) {
            reject_ocs("unsupported plan format or version");
        }
        if (root["endpoints"].as<int>() != npus_count) {
            reject_ocs("plan endpoint count does not match topology");
        }
        planes = root["planes"].as<int>();
        plane_states.resize(planes);
        plan_bandwidth = root["link_bandwidth_GBps"].as<double>();
        propagation_ns = root["propagation_ns"].as<double>();
        reconfiguration_ns = root["reconfiguration_ns"].as<double>();
        initial_reconfiguration = root["initial_reconfiguration"].as<bool>();
        if (root["logical_dimensions"] && root["logical_dimensions"].IsSequence()) {
            for (const auto& extent : root["logical_dimensions"]) {
                plan_dimensions.push_back(extent.as<int>());
            }
        }
        if (!root["assignments"] || !root["assignments"].IsSequence()) {
            reject_ocs("plan has no route assignments");
        }
        for (const auto& assignment : root["assignments"]) {
            const auto source = assignment["source"].as<int>();
            const auto destination = assignment["destination"].as<int>();
            const auto bytes = assignment["bytes"].as<uint64_t>();
            const auto stream = assignment["stream"].as<int>();
            const auto route = assignment["route"].as<std::string>();
            const auto request_id = assignment["request_id"]
                                        ? assignment["request_id"].as<int>()
                                        : -1;
            const auto not_before = assignment["not_before_ns"]
                                        ? assignment["not_before_ns"].as<EventTime>()
                                        : 0;
            if (source < 0 || source >= npus_count || destination < 0 ||
                destination >= npus_count || source == destination || bytes == 0 ||
                stream < 0 || request_id < -1 || not_before < 0) {
                reject_ocs("invalid route assignment");
            }
            const auto direct = route == "DIRECT";
            if (!direct && route.rfind("OCS", 0) != 0) {
                reject_ocs("invalid route assignment");
            }
            auto stripes = std::vector<PlaneStripe>();
            if (!direct) {
                if (!assignment["stripes"] || !assignment["stripes"].IsSequence()) {
                    reject_ocs("optical route assignment has no stripes");
                }
                auto striped_bytes = uint64_t{0};
                auto used_planes = std::set<int>();
                for (const auto& stripe : assignment["stripes"]) {
                    const auto plane = stripe["plane"].as<int>();
                    const auto stripe_bytes = stripe["bytes"].as<uint64_t>();
                    if (plane < 0 || plane >= planes || stripe_bytes == 0 ||
                        !used_planes.insert(plane).second) {
                        reject_ocs("invalid optical stripe");
                    }
                    stripes.push_back(PlaneStripe{plane, stripe_bytes});
                    striped_bytes += stripe_bytes;
                }
                if (striped_bytes != bytes) {
                    reject_ocs("optical stripes do not reconstruct logical bytes");
                }
            }
            route_assignments[{source, destination, bytes, stream}].push_back(
                RuntimeAssignment{direct, std::move(stripes), not_before, request_id});
            ++planned_assignments;
        }
        auto expected_round = 0;
        for (const auto& round_node : root["rounds"]) {
            const auto round = round_node["index"].as<int>();
            if (round != expected_round++) {
                reject_ocs("plan round indices are not contiguous");
            }
            const auto synchronize = round_node["synchronize"]
                                         ? round_node["synchronize"].as<bool>()
                                         : false;
            for (const auto& configuration_node : round_node["configurations"]) {
                auto configuration = Configuration{
                    configuration_node["plane"].as<int>(),
                    configuration_node["stream"].as<int>(),
                    configuration_node["force_reconfiguration"]
                        ? configuration_node["force_reconfiguration"].as<bool>()
                        : false,
                    round,
                    synchronize,
                    {}, {}};
                if (!configuration_node["matching"] ||
                    !configuration_node["matching"].IsSequence()) {
                    reject_ocs("configuration has no installed matching");
                }
                for (const auto& pair_node : configuration_node["matching"]) {
                    if (!pair_node.IsSequence() || pair_node.size() != 2) {
                        reject_ocs("invalid installed matching pair");
                    }
                    configuration.matching.emplace_back(
                        pair_node[0].as<int>(), pair_node[1].as<int>());
                }
                for (const auto& circuit_node : configuration_node["circuits"]) {
                    const auto source = circuit_node["source"].as<int>();
                    const auto destination = circuit_node["destination"].as<int>();
                    const auto bytes = circuit_node["bytes"].as<uint64_t>();
                    configuration.circuits.push_back(
                        Circuit{{source, destination}, bytes, bytes, false});
                    scheduled_bytes += bytes;
                }
                if (configuration.plane < 0 || configuration.plane >= planes) {
                    reject_ocs("configuration references an unavailable plane");
                }
                auto configuration_bytes = uint64_t{0};
                for (const auto& circuit : configuration.circuits) {
                    configuration_bytes = std::max(
                        configuration_bytes, circuit.bytes);
                }
                auto& state = plane_states[configuration.plane];
                state.data_time += positive_event_delay(
                    static_cast<double>(configuration_bytes) /
                    bw_GBps_to_Bpns(plan_bandwidth));
                state.configurations.push_back(std::move(configuration));
            }
        }
    } catch (const YAML::Exception& error) {
        reject_ocs(std::string("could not parse plan: ") + error.what());
    } catch (const std::exception& error) {
        reject_ocs(std::string("invalid plan value: ") + error.what());
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
    if (qtp_embedding && npus_count != 64) {
        reject_ocs("QTP embedding requires exactly 64 endpoints");
    }
    if (reconfiguration_ns < 0) {
        reject_ocs("reconfiguration latency must be nonnegative");
    }
    if (!plan_dimensions.empty()) {
        auto product = 1;
        for (const auto extent : plan_dimensions) {
            if (extent < 2) {
                reject_ocs("plan dimension extent must be at least two");
            }
            product *= extent;
        }
        if (product != npus_count) {
            reject_ocs("plan dimensions do not multiply to endpoint count");
        }
    }
    for (auto plane = 0; plane < planes; ++plane) {
        for (const auto& configuration : plane_states[plane].configurations) {
            if (configuration.plane != plane || configuration.stream < 0) {
                reject_ocs("invalid plane configuration");
            }
            auto sources = std::set<int>();
            auto destinations = std::set<int>();
            auto support = std::set<Pair>();
            for (const auto& pair : configuration.matching) {
                const auto [source, destination] = pair;
                if (source < 0 || source >= npus_count || destination < 0 ||
                    destination >= npus_count || source == destination ||
                    !sources.insert(source).second ||
                    !destinations.insert(destination).second) {
                    reject_ocs("invalid installed directional matching");
                }
                support.insert(pair);
            }
            if (support.empty()) {
                reject_ocs("installed matching must not be empty");
            }
            for (const auto& circuit : configuration.circuits) {
                if (circuit.bytes == 0 || support.find(circuit.pair) == support.end()) {
                    reject_ocs("transmitted circuit is outside the installed matching");
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
    const auto assignment = found->second.front();
    found->second.pop_front();
    dispatch_assignments[key].push_back(assignment);
    ++consumed_assignments;
    if (assignment.direct) {
        if (!base_torus) {
            reject_ocs("direct assignment requires a persistent base fabric");
        }
        return direct_route(src, dest);
    }
    if (assignment.stripes.empty()) {
        reject_ocs("optical assignment has no runtime stripes");
    }
    return ocs_route(src, dest, assignment.stripes.front().plane);
}

Route OcsSwitch::ocs_route(const DeviceId src, const DeviceId dest,
                           const int plane) const noexcept {
    auto path = Route();
    path.emplace_back(devices[src], to_switch_ports[plane][src]);
    path.emplace_back(devices[npus_count + plane], from_switch_ports[plane][dest]);
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
    const auto source = path.front().device->get_id();
    const auto destination = path.back().device->get_id();
    const auto key = AssignmentKey{source, destination, chunk->get_size(),
                                   chunk->get_stream()};
    const auto found = dispatch_assignments.find(key);
    if (found == dispatch_assignments.end() || found->second.empty()) {
        reject_ocs("route assignment was not preserved until dispatch");
    }
    const auto assignment = found->second.front();
    found->second.pop_front();

    if (!epoch_started) {
        epoch_started = true;
        epoch_start = event_queue->get_current_time();
        start_initial_planes();
    }
    const auto release = epoch_start + assignment.not_before;
    if (release > event_queue->get_current_time()) {
        auto* delayed = new DelayedChunk{this, std::move(chunk), assignment};
        event_queue->schedule_event(release, delayed_chunk_callback, delayed);
        return;
    }
    dispatch(std::move(chunk), assignment);
}

void OcsSwitch::delayed_chunk_callback(void* argument) noexcept {
    auto delayed = std::unique_ptr<DelayedChunk>(
        static_cast<DelayedChunk*>(argument));
    delayed->topology->dispatch(std::move(delayed->chunk), delayed->assignment);
}

void OcsSwitch::dispatch(std::unique_ptr<Chunk> chunk,
                         const RuntimeAssignment& assignment) noexcept {
    const auto& path = chunk->get_route();
    if (assignment.request_id >= 0) {
        ++causal_dispatches;
        const auto planned = epoch_start + assignment.not_before;
        const auto actual = event_queue->get_current_time();
        const auto slip = actual - planned;
        release_records.emplace_back(
            assignment.request_id, assignment.not_before, actual - epoch_start);
        if (slip > max_release_slip) {
            max_release_slip = slip;
            max_release_slip_request = assignment.request_id;
            max_release_slip_planned = assignment.not_before;
            max_release_slip_actual = actual - epoch_start;
        }
    }
    if (assignment.direct) {
        Topology::send(std::move(chunk));
        return;
    }
    const auto pair = Pair{path.front().device->get_id(), path.back().device->get_id()};
    if (assignment.stripes.empty()) {
        reject_ocs("optical dispatch has no stripes");
    }
    const auto logical_bytes = chunk->get_size();
    const auto stream = chunk->get_stream();
    const auto queued_at = event_queue->get_current_time();
    const auto transfer = std::make_shared<LogicalTransfer>(LogicalTransfer{
        std::move(chunk), pair, stream, queued_at,
        static_cast<int>(assignment.stripes.size())});
    auto maximum_stripe = ChunkSize{0};
    for (const auto& stripe : assignment.stripes) {
        maximum_stripe = std::max(maximum_stripe, stripe.bytes);
        pending[{stripe.plane, pair.first, pair.second, stream}].push_back(
            PendingStripe{transfer, stripe.bytes});
    }
    record_route_metrics(
        RouteClass::Switch, 1, 2, logical_bytes,
        positive_event_delay(propagation_ns),
        positive_event_delay(static_cast<double>(maximum_stripe) /
                             bw_GBps_to_Bpns(plan_bandwidth)));
    for (const auto& stripe : assignment.stripes) {
        try_start_transmissions(stripe.plane);
    }
}

void OcsSwitch::start_initial_planes() noexcept {
    for (auto plane = 0; plane < planes; ++plane) {
        auto& state = plane_states[plane];
        if (state.configurations.empty()) {
            continue;
        }
        if (!state.configurations[state.current].synchronize) {
            activate_configuration(plane);
        }
    }
    for (auto plane = 0; plane < planes; ++plane) {
        const auto& state = plane_states[plane];
        if (!state.configurations.empty() &&
            state.configurations[state.current].synchronize) {
            try_activate_synchronized_round(
                state.configurations[state.current].round);
        }
    }
}

void OcsSwitch::activate_configuration(const int plane) noexcept {
    auto& state = plane_states[plane];
    if (state.current >= state.configurations.size() || state.activated) {
        return;
    }
    const auto& configuration = state.configurations[state.current];
    const auto changed = state.has_installed_matching && matching_changed(
        state.installed_matching, configuration.matching);
    const auto charge = (!state.has_installed_matching
                             ? initial_reconfiguration
                             : configuration.force_reconfiguration || changed);
    state.installed_matching = configuration.matching;
    state.has_installed_matching = true;
    state.activated = true;
    if (!charge || reconfiguration_ns == 0) {
        try_start_transmissions(plane);
        if (configuration_complete(plane)) {
            advance_configuration(plane);
        }
        return;
    }
    state.reconfiguring = true;
    ++state.reconfigurations;
    ++reconfiguration_count;
    const auto delay = positive_event_delay(reconfiguration_ns);
    state.reconfiguration_time += delay;
    auto* callback = new PlaneCallback{this, plane};
    event_queue->schedule_event(event_queue->get_current_time() + delay,
                                reconfiguration_callback, callback);
}

void OcsSwitch::try_activate_synchronized_round(const int round) noexcept {
    auto participants = std::vector<int>();
    for (auto plane = 0; plane < planes; ++plane) {
        const auto& state = plane_states[plane];
        const auto planned = std::find_if(
            state.configurations.begin(), state.configurations.end(),
            [round](const Configuration& item) { return item.round == round; });
        if (planned == state.configurations.end()) {
            continue;
        }
        participants.push_back(plane);
        if (state.current >= state.configurations.size() ||
            state.configurations[state.current].round != round) {
            return;
        }
    }
    for (const auto plane : participants) {
        activate_configuration(plane);
    }
}

OcsSwitch::Circuit* OcsSwitch::find_circuit(const int plane,
                                            const Pair& pair) noexcept {
    auto& state = plane_states[plane];
    if (state.current >= state.configurations.size()) {
        return nullptr;
    }
    for (auto& circuit : state.configurations[state.current].circuits) {
        if (circuit.pair == pair) {
            return &circuit;
        }
    }
    return nullptr;
}

void OcsSwitch::try_start_transmissions(const int plane) noexcept {
    auto& state = plane_states[plane];
    if (state.reconfiguring || !state.activated ||
        state.current >= state.configurations.size()) {
        return;
    }
    auto progress = true;
    while (progress) {
        progress = false;
        auto& configuration = state.configurations[state.current];
        for (auto& circuit : configuration.circuits) {
            auto found = pending.find({plane, circuit.pair.first,
                                       circuit.pair.second, configuration.stream});
            if (circuit.busy || circuit.remaining == 0 || found == pending.end() ||
                found->second.empty()) {
                continue;
            }
            auto selected = std::move(found->second.front());
            found->second.pop_front();
            if (selected.bytes > circuit.remaining) {
                reject_ocs("stripe exceeds the remaining circuit byte quota");
            }
            start_transmission(plane, circuit, std::move(selected));
            progress = true;
        }
    }
}

void OcsSwitch::start_transmission(const int plane, Circuit& circuit,
                                   PendingStripe stripe) noexcept {
    circuit.busy = true;
    const auto bytes = stripe.bytes;
    const auto [source, destination] = circuit.pair;
    const auto switch_id = npus_count + plane;
    const auto source_port = to_switch_ports[plane][source];
    const auto destination_port = from_switch_ports[plane][destination];
    const auto serialization = positive_event_delay(
        static_cast<double>(bytes) / bw_GBps_to_Bpns(plan_bandwidth));
    const auto wait = event_queue->get_current_time() - stripe.transfer->queued_at;
    circuit_wait_time += wait;
    max_circuit_wait_time = std::max(max_circuit_wait_time, wait);
    ++circuit_transmissions;
    ++active_endpoint_planes[source][plane];
    ++active_endpoint_planes[destination][plane];
    ++active_endpoint_tx_planes[source][plane];
    ++active_endpoint_rx_planes[destination][plane];
    ++active_endpoint_peers[source][destination];
    ++active_endpoint_peers[destination][source];
    ++active_endpoint_tx_peers[source][destination];
    ++active_endpoint_rx_peers[destination][source];
    max_active_endpoint_ports[source] = std::max(
        max_active_endpoint_ports[source],
        static_cast<int>(active_endpoint_planes[source].size()));
    max_active_endpoint_ports[destination] = std::max(
        max_active_endpoint_ports[destination],
        static_cast<int>(active_endpoint_planes[destination].size()));
    max_active_endpoint_tx_ports[source] = std::max(
        max_active_endpoint_tx_ports[source],
        static_cast<int>(active_endpoint_tx_planes[source].size()));
    max_active_endpoint_rx_ports[destination] = std::max(
        max_active_endpoint_rx_ports[destination],
        static_cast<int>(active_endpoint_rx_planes[destination].size()));
    max_distinct_endpoint_peers[source] = std::max(
        max_distinct_endpoint_peers[source],
        static_cast<int>(active_endpoint_peers[source].size()));
    max_distinct_endpoint_peers[destination] = std::max(
        max_distinct_endpoint_peers[destination],
        static_cast<int>(active_endpoint_peers[destination].size()));
    max_distinct_endpoint_tx_peers[source] = std::max(
        max_distinct_endpoint_tx_peers[source],
        static_cast<int>(active_endpoint_tx_peers[source].size()));
    max_distinct_endpoint_rx_peers[destination] = std::max(
        max_distinct_endpoint_rx_peers[destination],
        static_cast<int>(active_endpoint_rx_peers[destination].size()));
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
        this, std::move(stripe.transfer), plane, circuit.pair, bytes};
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
    auto callback = std::unique_ptr<PlaneCallback>(
        static_cast<PlaneCallback*>(argument));
    auto& state = callback->topology->plane_states[callback->plane];
    state.reconfiguring = false;
    callback->topology->try_start_transmissions(callback->plane);
    if (callback->topology->configuration_complete(callback->plane)) {
        callback->topology->advance_configuration(callback->plane);
    }
}

void OcsSwitch::finish_serialization(Transmission* transmission) noexcept {
    auto* circuit = find_circuit(transmission->plane, transmission->pair);
    if (circuit == nullptr || !circuit->busy || circuit->remaining < transmission->bytes) {
        reject_ocs("serialization completed outside its installed circuit");
    }
    circuit->remaining -= transmission->bytes;
    circuit->busy = false;
    const auto [source, destination] = transmission->pair;
    if (--active_endpoint_planes[source][transmission->plane] == 0) {
        active_endpoint_planes[source].erase(transmission->plane);
    }
    if (--active_endpoint_planes[destination][transmission->plane] == 0) {
        active_endpoint_planes[destination].erase(transmission->plane);
    }
    if (--active_endpoint_tx_planes[source][transmission->plane] == 0) {
        active_endpoint_tx_planes[source].erase(transmission->plane);
    }
    if (--active_endpoint_rx_planes[destination][transmission->plane] == 0) {
        active_endpoint_rx_planes[destination].erase(transmission->plane);
    }
    if (--active_endpoint_peers[source][destination] == 0) {
        active_endpoint_peers[source].erase(destination);
    }
    if (--active_endpoint_peers[destination][source] == 0) {
        active_endpoint_peers[destination].erase(source);
    }
    if (--active_endpoint_tx_peers[source][destination] == 0) {
        active_endpoint_tx_peers[source].erase(destination);
    }
    if (--active_endpoint_rx_peers[destination][source] == 0) {
        active_endpoint_rx_peers[destination].erase(source);
    }
    transmitted_bytes += transmission->bytes;
    const auto propagation = positive_event_delay(propagation_ns);
    event_queue->schedule_event(event_queue->get_current_time() + propagation,
                                arrival_callback, transmission);
    try_start_transmissions(transmission->plane);
    if (configuration_complete(transmission->plane)) {
        advance_configuration(transmission->plane);
    }
}

void OcsSwitch::finish_arrival(Transmission* transmission) noexcept {
    auto& transfer = transmission->transfer;
    if (--transfer->remaining_stripes == 0) {
        transfer->chunk->invoke_callback();
    }
}

bool OcsSwitch::configuration_complete(const int plane) const noexcept {
    const auto& state = plane_states[plane];
    if (!state.activated || state.reconfiguring ||
        state.current >= state.configurations.size()) {
        return false;
    }
    for (const auto& circuit : state.configurations[state.current].circuits) {
        if (circuit.remaining != 0 || circuit.busy) {
            return false;
        }
    }
    return true;
}

bool OcsSwitch::matching_changed(const std::vector<Pair>& first,
                                 const std::vector<Pair>& second) const noexcept {
    auto first_matching = first;
    auto second_matching = second;
    std::sort(first_matching.begin(), first_matching.end());
    std::sort(second_matching.begin(), second_matching.end());
    return first_matching != second_matching;
}

void OcsSwitch::advance_configuration(const int plane) noexcept {
    auto& state = plane_states[plane];
    ++state.current;
    ++state.completed;
    state.activated = false;
    if (state.current >= state.configurations.size()) {
        return;
    }
    const auto& following = state.configurations[state.current];
    if (following.synchronize) {
        try_activate_synchronized_round(following.round);
    } else {
        activate_configuration(plane);
    }
}

int OcsSwitch::get_plane_count() const noexcept { return planes; }
int OcsSwitch::get_round_count() const noexcept {
    auto count = 0;
    for (const auto& state : plane_states) {
        count += static_cast<int>(state.configurations.size());
    }
    return count;
}
int OcsSwitch::get_completed_rounds() const noexcept {
    auto count = 0;
    for (const auto& state : plane_states) {
        count += state.completed;
    }
    return count;
}
int OcsSwitch::get_reconfiguration_count() const noexcept { return reconfiguration_count; }
uint64_t OcsSwitch::get_scheduled_bytes() const noexcept { return scheduled_bytes; }
uint64_t OcsSwitch::get_transmitted_bytes() const noexcept { return transmitted_bytes; }
EventTime OcsSwitch::get_reconfiguration_time() const noexcept {
    auto total = EventTime{0};
    for (const auto& state : plane_states) {
        total += state.reconfiguration_time;
    }
    return total;
}
EventTime OcsSwitch::get_max_plane_reconfiguration_time() const noexcept {
    auto result = EventTime{0};
    for (const auto& state : plane_states) {
        result = std::max(result, state.reconfiguration_time);
    }
    return result;
}
EventTime OcsSwitch::get_critical_plane_reconfiguration_time() const noexcept {
    if (plane_states.empty()) {
        return 0;
    }
    const auto critical = std::max_element(
        plane_states.begin(), plane_states.end(),
        [](const PlaneState& first, const PlaneState& second) {
            return first.reconfiguration_time + first.data_time <
                   second.reconfiguration_time + second.data_time;
        });
    return critical->reconfiguration_time;
}
EventTime OcsSwitch::get_plane_schedule_makespan() const noexcept {
    auto result = EventTime{0};
    for (const auto& state : plane_states) {
        result = std::max(
            result, state.reconfiguration_time + state.data_time);
    }
    return result;
}
EventTime OcsSwitch::get_circuit_wait_time() const noexcept {
    return circuit_wait_time;
}
EventTime OcsSwitch::get_max_circuit_wait_time() const noexcept {
    return max_circuit_wait_time;
}
uint64_t OcsSwitch::get_circuit_transmissions() const noexcept {
    return circuit_transmissions;
}
int OcsSwitch::get_max_active_ports(const DeviceId endpoint) const noexcept {
    assert(0 <= endpoint && endpoint < npus_count);
    return max_active_endpoint_ports[endpoint];
}
int OcsSwitch::get_max_distinct_peers(const DeviceId endpoint) const noexcept {
    assert(0 <= endpoint && endpoint < npus_count);
    return max_distinct_endpoint_peers[endpoint];
}

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
           << " rounds=" << get_round_count()
           << " completed_rounds=" << get_completed_rounds()
           << " reconfigurations=" << reconfiguration_count
           << " reconfiguration_ns=" << get_reconfiguration_time()
           << " max_plane_reconfiguration_ns="
           << get_max_plane_reconfiguration_time()
           << " critical_plane_reconfiguration_ns="
           << get_critical_plane_reconfiguration_time()
           << " plane_schedule_makespan_ns=" << get_plane_schedule_makespan()
           << " scheduled_bytes=" << scheduled_bytes
           << " transmitted_bytes=" << transmitted_bytes
           << " assignments=" << planned_assignments
           << " consumed_assignments=" << consumed_assignments
           << " circuit_wait_ns=" << circuit_wait_time
           << " max_circuit_wait_ns=" << max_circuit_wait_time
           << " circuit_transmissions=" << circuit_transmissions << '\n';
    for (auto endpoint = 0; endpoint < npus_count; ++endpoint) {
        output << "OCS_ENDPOINT endpoint=" << endpoint
               << " max_active_ports=" << max_active_endpoint_ports[endpoint]
               << " max_active_tx_ports="
               << max_active_endpoint_tx_ports[endpoint]
               << " max_active_rx_ports="
               << max_active_endpoint_rx_ports[endpoint]
               << " max_distinct_peers=" << max_distinct_endpoint_peers[endpoint]
               << " max_distinct_tx_peers="
               << max_distinct_endpoint_tx_peers[endpoint]
               << " max_distinct_rx_peers="
               << max_distinct_endpoint_rx_peers[endpoint]
               << " peak_injection_GBps="
               << max_active_endpoint_tx_ports[endpoint] * plan_bandwidth
               << " peak_receive_GBps="
               << max_active_endpoint_rx_ports[endpoint] * plan_bandwidth << '\n';
    }
    for (auto plane = 0; plane < planes; ++plane) {
        const auto& state = plane_states[plane];
        output << "OCS_PLANE plane=" << plane
               << " configurations=" << state.configurations.size()
               << " reconfigurations=" << state.reconfigurations
               << " reconfiguration_ns=" << state.reconfiguration_time
               << " data_ns=" << state.data_time
               << " finish_ns="
               << state.reconfiguration_time + state.data_time << '\n';
    }
    output << "OCS_REPLAY causal_dispatches=" << causal_dispatches
           << " max_release_slip_ns=" << max_release_slip
           << " request_id=" << max_release_slip_request
           << " planned_ns=" << max_release_slip_planned
           << " actual_ns=" << max_release_slip_actual << '\n';
    auto records = release_records;
    std::sort(records.begin(), records.end());
    for (const auto& [request_id, planned, actual] : records) {
        output << "OCS_RELEASE request_id=" << request_id
               << " planned_ns=" << planned
               << " actual_ns=" << actual
               << " slip_ns=" << actual - planned << '\n';
    }
}

const std::vector<int>& OcsSwitch::get_logical_to_physical() const noexcept {
    return logical_to_physical;
}

void OcsSwitch::build_qtp_embedding() noexcept {
    if (npus_count != 64 || width != 8) {
        reject_ocs("QTP embedding requires an 8x8 physical torus");
    }
    constexpr int qtp8[] = {0, 1, 4, 5, 7, 2, 3, 6};
    logical_to_physical.assign(npus_count, -1);
    physical_to_logical.assign(npus_count, -1);
    for (auto logical = 0; logical < npus_count; ++logical) {
        const auto a = logical % 4;
        const auto b = (logical / 4) % 2;
        const auto c = (logical / 8) % 4;
        const auto d = (logical / 32) % 2;
        const auto physical = qtp8[c + 4 * d] * 8 + qtp8[a + 4 * b];
        if (physical_to_logical[physical] != -1) {
            reject_ocs("QTP embedding is not a permutation");
        }
        logical_to_physical[logical] = physical;
        physical_to_logical[physical] = logical;
    }
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
            const auto physical = y * width + x;
            const auto node = physical_to_logical[physical];
            if (x + 1 < width) {
                remember(node, physical_to_logical[physical + 1]);
            } else {
                remember(node, physical_to_logical[y * width]);
            }
            if (y + 1 < width) {
                remember(node, physical_to_logical[physical + width]);
            } else {
                remember(node, physical_to_logical[x]);
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
    const auto source_physical = logical_to_physical[src];
    const auto destination_physical = logical_to_physical[dest];
    auto x = source_physical % width;
    auto y = source_physical / width;
    const auto destination_x = destination_physical % width;
    const auto destination_y = destination_physical / width;
    while (x != destination_x) {
        const auto next_x = step_towards(x, destination_x, width, (x & 1) != 0);
        const auto next = physical_to_logical[y * width + next_x];
        path.emplace_back(devices[current], base_ports.at({current, next}));
        current = next;
        x = next_x;
    }
    while (y != destination_y) {
        const auto next_y = step_towards(y, destination_y, width, (y & 1) != 0);
        const auto next = physical_to_logical[next_y * width + x];
        path.emplace_back(devices[current], base_ports.at({current, next}));
        current = next;
        y = next_y;
    }
    path.emplace_back(devices[dest]);
    return path;
}
