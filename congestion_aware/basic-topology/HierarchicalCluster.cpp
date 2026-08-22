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
    const Bandwidth bandwidth_per_nic,
    const Latency scale_out_latency,
    const int nic_count) noexcept
    : BasicTopology(npus_count,
                    npus_count + (npus_count / NodeSize) * (1 + nic_count) + 1,
                    scale_up_bandwidth, scale_up_latency),
      nodes_count(npus_count / NodeSize),
      nic_count_per_node(nic_count),
      bandwidth_per_nic(bandwidth_per_nic),
      scale_out_bandwidth_per_node(nic_count * bandwidth_per_nic),
      scale_out_latency(scale_out_latency),
      local_switch_base(npus_count),
      nic_base(npus_count + nodes_count),
      clos_id(npus_count + nodes_count + nodes_count * nic_count) {
    if (npus_count <= NodeSize || npus_count % NodeSize != 0) {
        reject_hierarchy("hierarchical cluster requires a multiple of eight NPUs");
    }
    if (bandwidth_per_nic <= 0 || scale_out_latency < 0 ||
        nic_count <= 0 || nic_count > NodeSize) {
        reject_hierarchy("hierarchical cluster requires positive scale-out bandwidth");
    }

    basic_topology_type = TopologyBuildingBlock::HierarchicalCluster;
    dims_count = 2;
    npus_count_per_dim = {NodeSize, nodes_count};
    bandwidth_per_dim = {
        scale_up_bandwidth,
        scale_out_bandwidth_per_node / static_cast<Bandwidth>(NodeSize)};

    for (auto npu = 0; npu < npus_count; ++npu) {
        const auto local = local_switch(node_of(npu));
        remember_bidirectional_port(
            npu, local,
            connect(npu, local, scale_up_bandwidth, scale_up_latency, true,
                    LinkClass::ScaleUp));
    }
    for (auto node = 0; node < nodes_count; ++node) {
        const auto local = local_switch(node);
        for (auto index = 0; index < nic_count_per_node; ++index) {
            const auto gateway = nic(node, index);
            remember_bidirectional_port(
                local, gateway,
                connect(local, gateway, bandwidth_per_nic,
                        scale_out_latency, true, LinkClass::Gateway));
            remember_bidirectional_port(
                gateway, clos_id,
                connect(gateway, clos_id, bandwidth_per_nic,
                        scale_out_latency, true, LinkClass::ScaleOut));
        }
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

DeviceId HierarchicalCluster::nic(const int node, const int index) const noexcept {
    assert(node >= 0 && node < nodes_count);
    assert(index >= 0 && index < nic_count_per_node);
    return nic_base + node * nic_count_per_node + index;
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

    const auto source_nic = nic(source_node, (src % NodeSize) % nic_count_per_node);
    const auto destination_nic = nic(
        destination_node, (dest % NodeSize) % nic_count_per_node);
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

int HierarchicalCluster::get_nic_count() const noexcept {
    return nic_count_per_node;
}

Bandwidth HierarchicalCluster::get_bandwidth_per_nic() const noexcept {
    return bandwidth_per_nic;
}

Bandwidth HierarchicalCluster::get_scale_out_bandwidth_per_node() const noexcept {
    return scale_out_bandwidth_per_node;
}
