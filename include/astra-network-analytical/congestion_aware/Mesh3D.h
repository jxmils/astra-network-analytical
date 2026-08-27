/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"
#include <array>

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/** Cubic 3-D mesh or torus with deterministic dimension-order routing. */
class Mesh3D final : public BasicTopology {
  public:
    Mesh3D(int npus_count, Bandwidth bandwidth, Latency latency,
           bool wraparound, bool topology_aware = false) noexcept;
    Mesh3D(std::array<int, 3> extents, Bandwidth bandwidth, Latency latency,
           bool wraparound, bool topology_aware = false) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

  private:
    int x_extent;
    int y_extent;
    int z_extent;
    bool wraparound;

    [[nodiscard]] int x_of(DeviceId id) const noexcept;
    [[nodiscard]] int y_of(DeviceId id) const noexcept;
    [[nodiscard]] int z_of(DeviceId id) const noexcept;
    [[nodiscard]] DeviceId id_of(int x, int y, int z) const noexcept;
    [[nodiscard]] int step_towards(int current, int target, int extent,
                                   bool tie_backward) const noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
