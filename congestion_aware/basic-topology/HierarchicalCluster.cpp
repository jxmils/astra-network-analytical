/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/HierarchicalCluster.h"
#include <cassert>
#include <cstdlib>
#include <iostream>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

namespace {

[[noreturn]] void reject_hierarchy(const char* message) noexcept {
    std::cerr << "[Error] (network/analytical/congestion_aware) " << message
              << std::endl;
    std::abort();
}

}  // namespace

HierarchicalCluster::HierarchicalCluster(
    const int npus_count, const Bandwidth scale_up_bandwidth,
    const Latency scale_up_latency,
    const Bandwidth scale_out_bandwidth_per_npu,
    const Latency scale_out_latency) noexcept
    : BasicTopology(npus_count,
                    npus_count + 2 * (npus_count / NodeSize) + 1,
                    scale_up_bandwidth, scale_up_latency),
      nodes_count(npus_count / NodeSize),
      scale_out_bandwidth_per_npu(scale_out_bandwidth_per_npu),
      scale_out_bandwidth_per_node(NodeSize * scale_out_bandwidth_per_npu),
      scale_out_latency(scale_out_latency),
      local_switch_base(npus_count),
      nic_base(npus_count + nodes_count),
      clos_id(npus_count + 2 * nodes_count) {
    if (npus_count <= NodeSize || npus_count % NodeSize != 0) {
        reject_hierarchy("hierarchical cluster requires a multiple of eight NPUs");
    }
    if (scale_out_bandwidth_per_npu <= 0 || scale_out_latency < 0) {
        reject_hierarchy("hierarchical cluster requires positive scale-out bandwidth");
    }

    basic_topology_type = TopologyBuildingBlock::HierarchicalCluster;
    dims_count = 2;
    npus_count_per_dim = {NodeSize, nodes_count};
    bandwidth_per_dim = {scale_up_bandwidth, scale_out_bandwidth_per_npu};

    for (auto npu = 0; npu < npus_count; ++npu) {
        const auto local = local_switch(node_of(npu));
        remember_bidirectional_port(
            npu, local,
            connect(npu, local, scale_up_bandwidth, scale_up_latency, true,
                    LinkClass::ScaleUp));
    }
    for (auto node = 0; node < nodes_count; ++node) {
        const auto local = local_switch(node);
        const auto gateway = nic(node);
        remember_bidirectional_port(
            local, gateway,
            connect(local, gateway, scale_out_bandwidth_per_node,
                    scale_out_latency, true, LinkClass::Gateway));
        remember_bidirectional_port(
            gateway, clos_id,
            connect(gateway, clos_id, scale_out_bandwidth_per_node,
                    scale_out_latency, true, LinkClass::ScaleOut));
    }
}

int HierarchicalCluster::node_of(const DeviceId npu) const noexcept {
    assert(npu >= 0 && npu < npus_count);
    return npu / NodeSize;
}

DeviceId HierarchicalCluster::local_switch(const int node) const noexcept {
    assert(node >= 0 && node < nodes_count);
    return local_switch_base + node;
}

DeviceId HierarchicalCluster::nic(const int node) const noexcept {
    assert(node >= 0 && node < nodes_count);
    return nic_base + node;
}

void HierarchicalCluster::remember_bidirectional_port(
    const DeviceId first, const DeviceId second,
    const std::pair<LinkId, LinkId>& connected_ports) noexcept {
    ports[{first, second}] = connected_ports.first;
    ports[{second, first}] = connected_ports.second;
}

Route HierarchicalCluster::route(const DeviceId src,
                                 const DeviceId dest) const noexcept {
    assert(src >= 0 && src < npus_count);
    assert(dest >= 0 && dest < npus_count);
    if (src == dest) {
        return Route({devices[src]});
    }

    const auto source_node = node_of(src);
    const auto destination_node = node_of(dest);
    const auto source_local = local_switch(source_node);
    const auto destination_local = local_switch(destination_node);
    auto path = Route();
    path.emplace_back(devices[src], ports.at({src, source_local}));
    if (source_node == destination_node) {
        path.emplace_back(devices[source_local], ports.at({source_local, dest}));
        path.emplace_back(devices[dest]);
        return path;
    }

    const auto source_nic = nic(source_node);
    const auto destination_nic = nic(destination_node);
    path.emplace_back(devices[source_local], ports.at({source_local, source_nic}));
    path.emplace_back(devices[source_nic], ports.at({source_nic, clos_id}));
    path.emplace_back(devices[clos_id], ports.at({clos_id, destination_nic}));
    path.emplace_back(devices[destination_nic],
                      ports.at({destination_nic, destination_local}));
    path.emplace_back(devices[destination_local], ports.at({destination_local, dest}));
    path.emplace_back(devices[dest]);
    return path;
}

int HierarchicalCluster::get_nodes_count() const noexcept {
    return nodes_count;
}

Bandwidth HierarchicalCluster::get_scale_out_bandwidth_per_npu() const noexcept {
    return scale_out_bandwidth_per_npu;
}

Bandwidth HierarchicalCluster::get_scale_out_bandwidth_per_node() const noexcept {
    return scale_out_bandwidth_per_node;
}
