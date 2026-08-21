/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"
#include <deque>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/**
 * Fixed-radix six-link comparison built on a square 2D mesh.
 *
 * Every NPU keeps the same base-mesh links and spends two additional ports on
 * either a dedicated row ring or two independent nonblocking switch planes.
 */
class Hybrid2D final : public BasicTopology {
  public:
    enum class ExtraFabric { RowRing, Switch };
    enum class RoutingPolicy { Static, Adaptive, DirectPreferredAdaptive, OfflineOracle };

    Hybrid2D(int npus_count, Bandwidth bandwidth, Latency latency,
             ExtraFabric extra_fabric, RoutingPolicy routing_policy,
             Bandwidth extra_bandwidth = -1.0,
             Latency extra_latency = -1.0,
             double direct_preference_factor = 1.10,
             std::string routing_plan_path = "",
             bool base_wraparound = false) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest,
                              ChunkSize chunk_size) const noexcept override;

    [[nodiscard]] ExtraFabric get_extra_fabric() const noexcept;
    [[nodiscard]] RoutingPolicy get_routing_policy() const noexcept;
    [[nodiscard]] double get_direct_preference_factor() const noexcept;

  private:
    struct SwitchPorts {
        LinkId to_switch;
        LinkId from_switch;
    };

    int width;
    int height;
    ExtraFabric extra_fabric;
    RoutingPolicy routing_policy;
    Bandwidth extra_bandwidth;
    Latency extra_latency;
    double direct_preference_factor;
    bool base_wraparound;
    using OfflineKey = std::tuple<DeviceId, DeviceId, ChunkSize>;
    mutable std::map<OfflineKey, std::deque<int>> offline_routes;
    std::map<std::pair<DeviceId, DeviceId>, LinkId> base_ports;
    std::map<std::pair<DeviceId, DeviceId>, LinkId> row_ring_ports;
    std::vector<std::vector<SwitchPorts>> switch_ports;

    [[nodiscard]] int x_of(DeviceId id) const noexcept;
    [[nodiscard]] int y_of(DeviceId id) const noexcept;
    [[nodiscard]] DeviceId id_of(int x, int y) const noexcept;
    [[nodiscard]] int step_towards(int current, int target, int extent,
                                   bool tie_backward) const noexcept;

    void build_base_mesh() noexcept;
    void build_row_rings() noexcept;
    void build_switch_planes() noexcept;
    void load_offline_plan(const std::string& path) noexcept;
    void remember_bidirectional_port(
        std::map<std::pair<DeviceId, DeviceId>, LinkId>& ports,
        DeviceId first, DeviceId second,
        const std::pair<LinkId, LinkId>& connected_ports) noexcept;

    [[nodiscard]] Route mesh_route(DeviceId src, DeviceId dest) const noexcept;
    [[nodiscard]] Route row_ring_route(DeviceId src, DeviceId dest) const noexcept;
    [[nodiscard]] Route switch_route(DeviceId src, DeviceId dest, int plane) const noexcept;
    [[nodiscard]] double path_cost(const Route& path, ChunkSize chunk_size,
                                   bool include_queue) const noexcept;
    [[nodiscard]] Route offline_route(DeviceId src, DeviceId dest,
                                      ChunkSize chunk_size) const noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
