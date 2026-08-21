/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/MultiPlaneSwitch.h"
#include "common/NetworkFunction.h"
#include <algorithm>
#include <cassert>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

MultiPlaneSwitch::MultiPlaneSwitch(const int npus_count,
                                   const Bandwidth bandwidth,
                                   const Latency latency) noexcept
    : BasicTopology(npus_count, npus_count + Planes, bandwidth, latency) {
    basic_topology_type = TopologyBuildingBlock::MultiSwitch6Adaptive;
    switch_ports.assign(Planes, std::vector<SwitchPorts>(npus_count));
    for (auto plane = 0; plane < Planes; ++plane) {
        const auto switch_id = npus_count + plane;
        for (auto npu = 0; npu < npus_count; ++npu) {
            const auto ports = connect(npu, switch_id, bandwidth, latency, true,
                                       LinkClass::SwitchUplink);
            switch_ports[plane][npu] = {ports.first, ports.second};
        }
    }
}

Route MultiPlaneSwitch::switch_route(const DeviceId src, const DeviceId dest,
                                     const int plane) const noexcept {
    assert(0 <= plane && plane < Planes);
    const auto switch_id = npus_count + plane;
    auto path = Route();
    path.emplace_back(devices[src], switch_ports[plane][src].to_switch);
    path.emplace_back(devices[switch_id], switch_ports[plane][dest].from_switch);
    path.emplace_back(devices[dest]);
    return path;
}

double MultiPlaneSwitch::path_cost(const Route& path,
                                   const ChunkSize chunk_size) const noexcept {
    auto latency_cost = 0.0;
    auto serialization_cost = 0.0;
    auto current = path.begin();
    auto next = std::next(current);
    while (next != path.end()) {
        const auto queued = current->device->get_outstanding_bytes(
            current->outgoing_link);
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

Route MultiPlaneSwitch::route(const DeviceId src, const DeviceId dest) const noexcept {
    return route(src, dest, 1);
}

Route MultiPlaneSwitch::route(const DeviceId src, const DeviceId dest,
                              const ChunkSize chunk_size) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);
    assert(chunk_size > 0);
    if (src == dest) {
        auto path = Route();
        path.emplace_back(devices[src]);
        return path;
    }

    const auto preferred_plane = (src + dest) % Planes;
    auto best = switch_route(src, dest, preferred_plane);
    auto best_cost = path_cost(best, chunk_size);
    for (auto offset = 1; offset < Planes; ++offset) {
        const auto plane = (preferred_plane + offset) % Planes;
        auto candidate = switch_route(src, dest, plane);
        const auto candidate_cost = path_cost(candidate, chunk_size);
        if (candidate_cost < best_cost) {
            best = std::move(candidate);
            best_cost = candidate_cost;
        }
    }
    return best;
}
