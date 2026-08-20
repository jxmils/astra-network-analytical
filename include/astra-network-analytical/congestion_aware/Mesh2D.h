/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"
#include <vector>

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/**
 * Mesh2D implements a 2D grid of NPUs with dimension-order (X-then-Y) routing.
 *
 * With wraparound=false this is a mesh; with wraparound=true it is a torus.
 * The two differ only by one extra edge per row and per column, so holding the
 * routing code identical isolates that single physical property.
 *
 * NPU ids are row-major: id = y * width + x.
 */
class Mesh2D final : public BasicTopology {
  public:
    /**
     * Logical-to-physical placement of NPU ids on the grid.
     *
     * RowMajor: id = y*width + x. A logical ring step crosses a row boundary
     *   16 times, each costing many hops on a mesh. This is a property of the
     *   *mapping*, not of the fabric.
     * Snake: a Hamiltonian cycle of the grid (valid because both extents are
     *   even), so every logical ring step is exactly one physical hop on a mesh
     *   and wraparound links are never needed.
     */
    enum class Embedding { RowMajor, Snake };

    /**
     * Constructor.
     *
     * @param npus_count number of NPUs; must be a perfect square
     * @param bandwidth bandwidth of each link
     * @param latency latency of each link
     * @param wraparound if true, build a torus; if false, a mesh
     */
    Mesh2D(int npus_count, Bandwidth bandwidth, Latency latency, bool wraparound,
           Embedding embedding = Embedding::RowMajor) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;

  private:
    /// grid extent (width == height; npus_count must be a perfect square)
    int width;
    int height;

    /// true for torus, false for mesh
    bool wraparound;

    [[nodiscard]] int x_of(DeviceId id) const noexcept;
    [[nodiscard]] int y_of(DeviceId id) const noexcept;
    [[nodiscard]] DeviceId id_of(int x, int y) const noexcept;

    /// Advance one coordinate step from cur towards target along an axis of
    /// extent n, taking the shorter direction when wraparound is enabled.
    ///
    /// On a wrapped axis of even extent the antipodal distance n/2 is an exact
    /// tie. Always resolving it the same way piles every antipodal flow onto
    /// one direction: for a full all-to-all on a 16-ring that loads clockwise
    /// links to 36 traversals against 28 counterclockwise, a 12.5% overload of
    /// the bottleneck. `tie_backward` splits those flows between directions.
    [[nodiscard]] int step_towards(int cur, int target, int n, bool tie_backward) const noexcept;

    /// logical NPU id -> physical grid device id
    std::vector<DeviceId> placement;

    /// build `placement` for the requested embedding
    void build_placement(Embedding embedding) noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
