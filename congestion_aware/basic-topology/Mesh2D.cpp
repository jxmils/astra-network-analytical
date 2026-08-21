/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Mesh2D.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

namespace {

[[noreturn]] void reject_mesh_configuration(const char* message) noexcept {
    std::cerr << "[Error] (network/analytical/congestion_aware) " << message << std::endl;
    std::abort();
}

}  // namespace

Mesh2D::Mesh2D(const int npus_count, const Bandwidth bandwidth, const Latency latency,
               const bool wraparound, const Embedding embedding) noexcept
    : wraparound(wraparound),
      BasicTopology(npus_count, npus_count, bandwidth, latency) {
    if (npus_count <= 0) {
        reject_mesh_configuration("Mesh2D/Torus2D requires a positive npus_count");
    }
    if (bandwidth <= 0) {
        reject_mesh_configuration("Mesh2D/Torus2D requires positive bandwidth");
    }
    if (latency < 0) {
        reject_mesh_configuration("Mesh2D/Torus2D requires non-negative latency");
    }

    // square grid only, for now
    const auto side = static_cast<int>(std::lround(std::sqrt(static_cast<double>(npus_count))));
    if (side * side != npus_count) {
        reject_mesh_configuration("Mesh2D/Torus2D requires a perfect-square npus_count");
    }
    // The snake placement is a Hamiltonian cycle of the grid, which this
    // construction only closes for even extents. Guard it rather than
    // silently emitting a non-cycle on an odd grid.
    if (embedding == Embedding::Snake && side % 2 != 0) {
        reject_mesh_configuration("Snake placement requires an even grid extent");
    }
    width = side;
    height = side;

    basic_topology_type = wraparound ? TopologyBuildingBlock::Torus2D : TopologyBuildingBlock::Mesh2D;

    build_placement(embedding);

    // X edges: connect (x,y) -> (x+1,y). Links are bidirectional, so one call per edge.
    for (auto y = 0; y < height; y++) {
        for (auto x = 0; x + 1 < width; x++) {
            connect(id_of(x, y), id_of(x + 1, y), bandwidth, latency, true,
                    LinkClass::BaseMesh);
        }
        if (wraparound && width > 2) {
            connect(id_of(width - 1, y), id_of(0, y), bandwidth, latency, true,
                    LinkClass::BaseMesh);
        }
    }

    // Y edges: connect (x,y) -> (x,y+1)
    for (auto x = 0; x < width; x++) {
        for (auto y = 0; y + 1 < height; y++) {
            connect(id_of(x, y), id_of(x, y + 1), bandwidth, latency, true,
                    LinkClass::BaseMesh);
        }
        if (wraparound && height > 2) {
            connect(id_of(x, height - 1), id_of(x, 0), bandwidth, latency, true,
                    LinkClass::BaseMesh);
        }
    }
}

int Mesh2D::x_of(const DeviceId id) const noexcept {
    return static_cast<int>(id) % width;
}

int Mesh2D::y_of(const DeviceId id) const noexcept {
    return static_cast<int>(id) / width;
}

DeviceId Mesh2D::id_of(const int x, const int y) const noexcept {
    return static_cast<DeviceId>(y * width + x);
}

int Mesh2D::step_towards(const int cur, const int target, const int n,
                         const bool tie_backward) const noexcept {
    if (!wraparound) {
        return (target > cur) ? (cur + 1) : (cur - 1);
    }

    // shorter direction around the ring
    const auto forward = ((target - cur) + n) % n;
    const auto backward = n - forward;
    if (forward < backward) {
        return (cur + 1) % n;
    }
    if (backward < forward) {
        return (cur - 1 + n) % n;
    }

    // exact tie: antipodal. Resolving every tie the same way overloads that
    // direction by 12.5% under all-to-all; split them instead.
    return tie_backward ? ((cur - 1 + n) % n) : ((cur + 1) % n);
}

Route Mesh2D::route(const DeviceId src, const DeviceId dest) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    auto route = Route();

    // translate logical NPU ids to physical grid positions
    const auto phys_src = placement[src];
    const auto phys_dest = placement[dest];

    auto cx = x_of(phys_src);
    auto cy = y_of(phys_src);
    const auto dx = x_of(phys_dest);
    const auto dy = y_of(phys_dest);

    route.push_back(devices[id_of(cx, cy)]);

    // Split antipodal ties by source parity so both directions carry half.
    // Deterministic, so routes stay stable across runs.
    const auto tie_backward = ((static_cast<int>(phys_src) & 1) != 0);

    // dimension-order routing: X first, then Y
    while (cx != dx) {
        cx = step_towards(cx, dx, width, tie_backward);
        route.push_back(devices[id_of(cx, cy)]);
    }
    while (cy != dy) {
        cy = step_towards(cy, dy, height, tie_backward);
        route.push_back(devices[id_of(cx, cy)]);
    }

    return route;
}

void Mesh2D::build_placement(const Embedding embedding) noexcept {
    placement.clear();
    placement.reserve(npus_count);

    if (embedding == Embedding::RowMajor) {
        for (auto i = 0; i < npus_count; i++) {
            placement.push_back(static_cast<DeviceId>(i));
        }
        return;
    }

    // Snake: Hamiltonian cycle of the grid. Row 0 left-to-right, then rows
    // 1..h-1 snaking through columns 1..w-1, then back up column 0. Consecutive
    // entries are always grid-adjacent, and the last is adjacent to the first,
    // so a logical ring maps to single physical hops throughout.
    for (auto x = 0; x < width; x++) {
        placement.push_back(id_of(x, 0));
    }
    for (auto y = 1; y < height; y++) {
        if (y % 2 == 1) {
            for (auto x = width - 1; x >= 1; x--) {
                placement.push_back(id_of(x, y));
            }
        } else {
            for (auto x = 1; x < width; x++) {
                placement.push_back(id_of(x, y));
            }
        }
    }
    for (auto y = height - 1; y >= 1; y--) {
        placement.push_back(id_of(0, y));
    }

    assert(static_cast<int>(placement.size()) == npus_count);
}
