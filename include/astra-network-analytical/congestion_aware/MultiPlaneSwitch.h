/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"
#include <vector>

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/** Six independent nonblocking switch planes, one physical port per plane. */
class MultiPlaneSwitch final : public BasicTopology {
  public:
    static constexpr int Planes = 6;

    MultiPlaneSwitch(int npus_count, Bandwidth bandwidth, Latency latency) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest,
                              ChunkSize chunk_size) const noexcept override;

  private:
    struct SwitchPorts {
        LinkId to_switch;
        LinkId from_switch;
    };

    std::vector<std::vector<SwitchPorts>> switch_ports;

    [[nodiscard]] Route switch_route(DeviceId src, DeviceId dest,
                                     int plane) const noexcept;
    [[nodiscard]] double path_cost(const Route& path,
                                   ChunkSize chunk_size) const noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
