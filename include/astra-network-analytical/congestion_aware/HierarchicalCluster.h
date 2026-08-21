/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"
#include <map>
#include <utility>

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/**
 * Two-level 8-NPU-node cluster with a nonblocking local fabric and Clos core.
 *
 * The supplied extra bandwidth is effective scale-out bandwidth per NPU. Each
 * node gateway and Clos port therefore receives NodeSize times that bandwidth.
 */
class HierarchicalCluster final : public BasicTopology {
  public:
    static constexpr int NodeSize = 8;

    HierarchicalCluster(int npus_count, Bandwidth scale_up_bandwidth,
                        Latency scale_up_latency,
                        Bandwidth scale_out_bandwidth_per_npu,
                        Latency scale_out_latency) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

    [[nodiscard]] int get_nodes_count() const noexcept;
    [[nodiscard]] Bandwidth get_scale_out_bandwidth_per_npu() const noexcept;
    [[nodiscard]] Bandwidth get_scale_out_bandwidth_per_node() const noexcept;

  private:
    int nodes_count;
    Bandwidth scale_out_bandwidth_per_npu;
    Bandwidth scale_out_bandwidth_per_node;
    Latency scale_out_latency;
    DeviceId local_switch_base;
    DeviceId nic_base;
    DeviceId clos_id;
    std::map<std::pair<DeviceId, DeviceId>, LinkId> ports;

    [[nodiscard]] int node_of(DeviceId npu) const noexcept;
    [[nodiscard]] DeviceId local_switch(int node) const noexcept;
    [[nodiscard]] DeviceId nic(int node) const noexcept;
    void remember_bidirectional_port(
        DeviceId first, DeviceId second,
        const std::pair<LinkId, LinkId>& connected_ports) noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
