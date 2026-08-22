/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Hybrid2D.h"
#include "common/NetworkFunction.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

namespace {

[[noreturn]] void reject_hybrid_configuration(const std::string& message) noexcept {
    std::cerr << "[Error] (network/analytical/congestion_aware) " << message << std::endl;
    std::abort();
}

}  // namespace

Hybrid2D::Hybrid2D(const int npus_count, const Bandwidth bandwidth,
                   const Latency latency, const ExtraFabric extra_fabric,
                   const RoutingPolicy routing_policy,
                   const Bandwidth requested_extra_bandwidth,
                   const Latency requested_extra_latency,
                   const double direct_preference_factor,
                   std::string routing_plan_path,
                   const bool base_wraparound,
                   const LogicalShape logical_shape) noexcept
    : BasicTopology(npus_count,
                    npus_count + (extra_fabric == ExtraFabric::Switch ? 2 : 0),
                    bandwidth, latency),
      width(static_cast<int>(std::lround(std::sqrt(static_cast<double>(npus_count))))),
      height(width),
      extra_fabric(extra_fabric),
      routing_policy(routing_policy),
      extra_bandwidth(requested_extra_bandwidth > 0 ? requested_extra_bandwidth
                                                    : bandwidth),
      extra_latency(requested_extra_latency >= 0 ? requested_extra_latency
                                                 : latency),
      direct_preference_factor(direct_preference_factor),
      base_wraparound(base_wraparound),
      logical_shape(logical_shape) {
    if (width * height != npus_count) {
        reject_hybrid_configuration("hybrid topology requires a perfect-square npus_count");
    }
    if (width < 3) {
        reject_hybrid_configuration("hybrid topology requires a grid extent of at least three");
    }
    if (direct_preference_factor < 1.0) {
        reject_hybrid_configuration("direct preference factor must be at least one");
    }
    if (base_wraparound &&
        (extra_fabric != ExtraFabric::Switch ||
         (routing_policy != RoutingPolicy::Adaptive &&
          routing_policy != RoutingPolicy::DirectOnly &&
          routing_policy != RoutingPolicy::SwitchOnly))) {
        reject_hybrid_configuration(
            "wraparound hybrid requires adaptive or forced switch routing");
    }
    if (logical_shape == LogicalShape::Grid &&
        (!base_wraparound || extra_fabric != ExtraFabric::Switch ||
         (routing_policy != RoutingPolicy::Adaptive &&
          routing_policy != RoutingPolicy::DirectOnly &&
          routing_policy != RoutingPolicy::SwitchOnly))) {
        reject_hybrid_configuration(
            "grid logical dimensions require a torus-switch fabric");
    }

    if (logical_shape == LogicalShape::Grid) {
        dims_count = 2;
        npus_count_per_dim = {width, height};
        bandwidth_per_dim = {bandwidth, bandwidth};
    }

    if (extra_fabric == ExtraFabric::RowRing) {
        if (routing_policy != RoutingPolicy::Static &&
            routing_policy != RoutingPolicy::Adaptive) {
            reject_hybrid_configuration(
                "direct-preferred and offline routing require the switch fabric");
        }
        basic_topology_type = routing_policy == RoutingPolicy::Adaptive
                                  ? TopologyBuildingBlock::MeshRowRingAdaptive
                                  : TopologyBuildingBlock::MeshRowRing;
    } else if (base_wraparound) {
        if (routing_policy == RoutingPolicy::DirectOnly) {
            basic_topology_type =
                logical_shape == LogicalShape::Grid
                    ? TopologyBuildingBlock::TorusSwitchDirectOnly2D
                    : TopologyBuildingBlock::TorusSwitchDirectOnly;
        } else if (routing_policy == RoutingPolicy::SwitchOnly) {
            basic_topology_type =
                logical_shape == LogicalShape::Grid
                    ? TopologyBuildingBlock::TorusSwitchSwitchOnly2D
                    : TopologyBuildingBlock::TorusSwitchSwitchOnly;
        } else {
            basic_topology_type =
                logical_shape == LogicalShape::Grid
                    ? TopologyBuildingBlock::TorusSwitchAdaptive2D
                    : TopologyBuildingBlock::TorusSwitchAdaptive;
        }
    } else {
        switch (routing_policy) {
        case RoutingPolicy::Static:
            basic_topology_type = TopologyBuildingBlock::MeshSwitch;
            break;
        case RoutingPolicy::Adaptive:
            basic_topology_type = TopologyBuildingBlock::MeshSwitchAdaptive;
            break;
        case RoutingPolicy::DirectPreferredAdaptive:
            basic_topology_type = TopologyBuildingBlock::MeshSwitchDirectPreferred;
            break;
        case RoutingPolicy::OfflineOracle:
            basic_topology_type = TopologyBuildingBlock::MeshSwitchOfflineOracle;
            break;
        case RoutingPolicy::DirectOnly:
        case RoutingPolicy::SwitchOnly:
            reject_hybrid_configuration(
                "forced routing policies require the wraparound hybrid");
        }
    }

    build_base_mesh();
    if (extra_fabric == ExtraFabric::RowRing) {
        build_row_rings();
    } else {
        build_switch_planes();
    }
    if (routing_policy == RoutingPolicy::OfflineOracle) {
        load_offline_plan(routing_plan_path);
    }
}

int Hybrid2D::x_of(const DeviceId id) const noexcept {
    return id % width;
}

int Hybrid2D::y_of(const DeviceId id) const noexcept {
    return id / width;
}

DeviceId Hybrid2D::id_of(const int x, const int y) const noexcept {
    return y * width + x;
}

int Hybrid2D::step_towards(const int current, const int target, const int extent,
                           const bool tie_backward) const noexcept {
    if (!base_wraparound) {
        return target > current ? current + 1 : current - 1;
    }
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

void Hybrid2D::remember_bidirectional_port(
    std::map<std::pair<DeviceId, DeviceId>, LinkId>& ports,
    const DeviceId first, const DeviceId second,
    const std::pair<LinkId, LinkId>& connected_ports) noexcept {
    ports[{first, second}] = connected_ports.first;
    ports[{second, first}] = connected_ports.second;
}

void Hybrid2D::build_base_mesh() noexcept {
    for (auto y = 0; y < height; ++y) {
        for (auto x = 0; x + 1 < width; ++x) {
            const auto first = id_of(x, y);
            const auto second = id_of(x + 1, y);
            remember_bidirectional_port(
                base_ports, first, second,
                connect(first, second, bandwidth, latency, true, LinkClass::BaseMesh));
        }
        if (base_wraparound) {
            const auto first = id_of(width - 1, y);
            const auto second = id_of(0, y);
            remember_bidirectional_port(
                base_ports, first, second,
                connect(first, second, bandwidth, latency, true,
                        LinkClass::BaseMesh));
        }
    }
    for (auto x = 0; x < width; ++x) {
        for (auto y = 0; y + 1 < height; ++y) {
            const auto first = id_of(x, y);
            const auto second = id_of(x, y + 1);
            remember_bidirectional_port(
                base_ports, first, second,
                connect(first, second, bandwidth, latency, true, LinkClass::BaseMesh));
        }
        if (base_wraparound) {
            const auto first = id_of(x, height - 1);
            const auto second = id_of(x, 0);
            remember_bidirectional_port(
                base_ports, first, second,
                connect(first, second, bandwidth, latency, true,
                        LinkClass::BaseMesh));
        }
    }
}

void Hybrid2D::build_row_rings() noexcept {
    for (auto y = 0; y < height; ++y) {
        for (auto x = 0; x < width; ++x) {
            const auto first = id_of(x, y);
            const auto second = id_of((x + 1) % width, y);
            remember_bidirectional_port(
                row_ring_ports, first, second,
                connect(first, second, extra_bandwidth, extra_latency, true,
                        LinkClass::RowRing));
        }
    }
}

void Hybrid2D::build_switch_planes() noexcept {
    switch_ports.assign(2, std::vector<SwitchPorts>(npus_count));
    for (auto plane = 0; plane < 2; ++plane) {
        const auto switch_id = npus_count + plane;
        for (auto npu = 0; npu < npus_count; ++npu) {
            const auto ports = connect(npu, switch_id, extra_bandwidth, extra_latency, true,
                                       LinkClass::SwitchUplink);
            switch_ports[plane][npu] = {ports.first, ports.second};
        }
    }
}

void Hybrid2D::load_offline_plan(const std::string& path) noexcept {
    if (path.empty()) {
        reject_hybrid_configuration("offline routing requires a routing_plan file");
    }
    auto input = std::ifstream(path);
    if (!input) {
        reject_hybrid_configuration("could not open offline routing plan: " + path);
    }

    auto line = std::string();
    auto line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.empty() || line.front() == '#') {
            continue;
        }
        auto parser = std::istringstream(line);
        auto src = DeviceId();
        auto dest = DeviceId();
        auto chunk_size = ChunkSize();
        auto route_name = std::string();
        auto trailing = std::string();
        if (!(parser >> src >> dest >> chunk_size >> route_name) || parser >> trailing) {
            reject_hybrid_configuration(
                "invalid offline routing-plan line " + std::to_string(line_number));
        }
        if (src < 0 || src >= npus_count || dest < 0 || dest >= npus_count ||
            src == dest || chunk_size == 0) {
            reject_hybrid_configuration(
                "invalid offline routing-plan request on line " +
                std::to_string(line_number));
        }
        auto route_id = -1;
        if (route_name == "DIRECT") {
            route_id = 0;
        } else if (route_name == "SWITCH0") {
            route_id = 1;
        } else if (route_name == "SWITCH1") {
            route_id = 2;
        } else {
            reject_hybrid_configuration(
                "invalid offline route on line " + std::to_string(line_number));
        }
        offline_routes[{src, dest, chunk_size}].push_back(route_id);
    }
    if (offline_routes.empty()) {
        reject_hybrid_configuration("offline routing plan contains no requests");
    }
}

Route Hybrid2D::mesh_route(const DeviceId src, const DeviceId dest) const noexcept {
    auto path = Route();
    auto current = src;
    auto x = x_of(src);
    auto y = y_of(src);
    const auto destination_x = x_of(dest);
    const auto destination_y = y_of(dest);
    const auto x_tie_backward = (x & 1) != 0;
    const auto y_tie_backward = (y & 1) != 0;

    while (x != destination_x) {
        const auto next_x = step_towards(x, destination_x, width,
                                         x_tie_backward);
        const auto next = id_of(next_x, y);
        path.emplace_back(devices[current], base_ports.at({current, next}));
        current = next;
        x = next_x;
    }
    while (y != destination_y) {
        const auto next_y = step_towards(y, destination_y, height,
                                         y_tie_backward);
        const auto next = id_of(x, next_y);
        path.emplace_back(devices[current], base_ports.at({current, next}));
        current = next;
        y = next_y;
    }
    path.emplace_back(devices[dest]);
    return path;
}

Route Hybrid2D::row_ring_route(const DeviceId src, const DeviceId dest) const noexcept {
    auto path = Route();
    auto current = src;
    auto x = x_of(src);
    auto y = y_of(src);
    const auto destination_x = x_of(dest);
    const auto destination_y = y_of(dest);

    const auto forward = (destination_x - x + width) % width;
    const auto backward = width - forward;
    auto direction = 1;
    if (backward < forward || (backward == forward && (src & 1) != 0)) {
        direction = -1;
    }
    while (x != destination_x) {
        const auto next_x = (x + direction + width) % width;
        const auto next = id_of(next_x, y);
        path.emplace_back(devices[current], row_ring_ports.at({current, next}));
        current = next;
        x = next_x;
    }
    while (y != destination_y) {
        const auto next_y = y + (destination_y > y ? 1 : -1);
        const auto next = id_of(x, next_y);
        path.emplace_back(devices[current], base_ports.at({current, next}));
        current = next;
        y = next_y;
    }
    path.emplace_back(devices[dest]);
    return path;
}

Route Hybrid2D::switch_route(const DeviceId src, const DeviceId dest,
                             const int plane) const noexcept {
    assert(plane == 0 || plane == 1);
    const auto switch_id = npus_count + plane;
    auto path = Route();
    path.emplace_back(devices[src], switch_ports[plane][src].to_switch);
    path.emplace_back(devices[switch_id], switch_ports[plane][dest].from_switch);
    path.emplace_back(devices[dest]);
    return path;
}

Route Hybrid2D::offline_route(const DeviceId src, const DeviceId dest,
                              const ChunkSize chunk_size) const noexcept {
    const auto key = OfflineKey{src, dest, chunk_size};
    const auto found = offline_routes.find(key);
    if (found == offline_routes.end() || found->second.empty()) {
        reject_hybrid_configuration(
            "offline routing plan has no remaining assignment for " +
            std::to_string(src) + " -> " + std::to_string(dest) +
            " size " + std::to_string(chunk_size));
    }
    const auto route_id = found->second.front();
    found->second.pop_front();
    if (route_id == 0) {
        return mesh_route(src, dest);
    }
    return switch_route(src, dest, route_id - 1);
}

double Hybrid2D::path_cost(const Route& path, const ChunkSize chunk_size,
                           const bool include_queue) const noexcept {
    if (path.size() <= 1) {
        return 0.0;
    }

    auto latency_cost = 0.0;
    auto serialization_cost = 0.0;
    auto current = path.begin();
    auto next = std::next(current);
    while (next != path.end()) {
        assert(current->outgoing_link != AutomaticLink);
        assert(current->device->link_connects(current->outgoing_link,
                                              next->device->get_id()));
        const auto queued = include_queue
                                ? current->device->get_outstanding_bytes(
                                      current->outgoing_link)
                                : 0;
        const auto bandwidth_Bpns = bw_GBps_to_Bpns(
            current->device->get_link_bandwidth(current->outgoing_link));
        latency_cost += current->device->get_link_latency(current->outgoing_link);
        serialization_cost = std::max(
            serialization_cost,
            (static_cast<double>(queued) + static_cast<double>(chunk_size)) /
                bandwidth_Bpns);
        ++current;
        ++next;
    }
    return latency_cost + serialization_cost;
}

Route Hybrid2D::route(const DeviceId src, const DeviceId dest) const noexcept {
    return route(src, dest, 1);
}

Route Hybrid2D::route(const DeviceId src, const DeviceId dest,
                      const ChunkSize chunk_size) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);
    assert(chunk_size > 0);

    const auto direct = mesh_route(src, dest);
    if (src == dest) {
        return direct;
    }
    if (routing_policy == RoutingPolicy::OfflineOracle) {
        return offline_route(src, dest, chunk_size);
    }
    if (routing_policy == RoutingPolicy::DirectOnly) {
        return direct;
    }

    const auto include_queue = routing_policy != RoutingPolicy::Static;
    const auto direct_cost = path_cost(direct, chunk_size, include_queue);

    if (extra_fabric == ExtraFabric::RowRing) {
        const auto extra = row_ring_route(src, dest);
        const auto extra_cost = path_cost(extra, chunk_size, include_queue);
        if (extra_cost < direct_cost) {
            return extra;
        }
        if (direct_cost < extra_cost) {
            return direct;
        }
        return (src & 1) == 0 ? direct : extra;
    }

    const auto preferred_plane = (src + dest) & 1;
    const auto preferred_switch = switch_route(src, dest, preferred_plane);
    const auto other_switch = switch_route(src, dest, 1 - preferred_plane);
    const auto preferred_cost = path_cost(preferred_switch, chunk_size, include_queue);
    const auto other_cost = path_cost(other_switch, chunk_size, include_queue);

    auto best_switch_cost = preferred_cost;
    auto best_switch = preferred_switch;
    if (other_cost < best_switch_cost) {
        best_switch_cost = other_cost;
        best_switch = other_switch;
    }
    if (routing_policy == RoutingPolicy::SwitchOnly) {
        return best_switch;
    }
    if (routing_policy == RoutingPolicy::DirectPreferredAdaptive) {
        return direct_cost <= direct_preference_factor * best_switch_cost
                   ? direct
                   : best_switch;
    }

    auto best_cost = direct_cost;
    auto best = direct;
    if (preferred_cost < best_cost) {
        best_cost = preferred_cost;
        best = preferred_switch;
    }
    if (other_cost < best_cost) {
        best = other_switch;
    }
    return best;
}

Hybrid2D::ExtraFabric Hybrid2D::get_extra_fabric() const noexcept {
    return extra_fabric;
}

Hybrid2D::RoutingPolicy Hybrid2D::get_routing_policy() const noexcept {
    return routing_policy;
}

double Hybrid2D::get_direct_preference_factor() const noexcept {
    return direct_preference_factor;
}
