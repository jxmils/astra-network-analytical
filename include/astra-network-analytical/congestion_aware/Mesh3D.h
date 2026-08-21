/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/** Cubic 3-D mesh or torus with deterministic dimension-order routing. */
class Mesh3D final : public BasicTopology {
  public:
    Mesh3D(int npus_count, Bandwidth bandwidth, Latency latency,
           bool wraparound) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

  private:
    int extent;
    bool wraparound;

    [[nodiscard]] int x_of(DeviceId id) const noexcept;
    [[nodiscard]] int y_of(DeviceId id) const noexcept;
    [[nodiscard]] int z_of(DeviceId id) const noexcept;
    [[nodiscard]] DeviceId id_of(int x, int y, int z) const noexcept;
    [[nodiscard]] int step_towards(int current, int target,
                                   bool tie_backward) const noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
