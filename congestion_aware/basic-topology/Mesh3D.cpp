/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Mesh3D.h"
#include <array>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

namespace {

[[noreturn]] void reject_mesh3d_configuration(const char* message) noexcept {
    std::cerr << "[Error] (network/analytical/congestion_aware) " << message << std::endl;
    std::abort();
}

int product(const std::array<int, 3>& extents) noexcept {
    return extents[0] * extents[1] * extents[2];
}

std::array<int, 3> cubic_extents(const int npus_count) noexcept {
    const auto extent = static_cast<int>(std::lround(std::cbrt(
        static_cast<double>(npus_count))));
    if (extent * extent * extent != npus_count) {
        reject_mesh3d_configuration("Mesh3D/Torus3D requires a perfect-cube npus_count");
    }
    return {extent, extent, extent};
}

void validate_extents(const std::array<int, 3>& extents,
                      const bool wraparound) noexcept {
    for (const auto extent : extents) {
        if (extent <= 1) {
            reject_mesh3d_configuration("Mesh3D/Torus3D extents must be larger than one");
        }
        if (wraparound && extent <= 2) {
            reject_mesh3d_configuration(
                "Torus3D requires every extent greater than two for distinct neighbors");
        }
    }
}

}  // namespace

Mesh3D::Mesh3D(const int npus_count, const Bandwidth bandwidth,
               const Latency latency, const bool wraparound,
               const bool topology_aware) noexcept
    : Mesh3D(cubic_extents(npus_count), bandwidth, latency, wraparound,
             topology_aware) {}

Mesh3D::Mesh3D(const std::array<int, 3> extents, const Bandwidth bandwidth,
               const Latency latency, const bool wraparound,
               const bool topology_aware) noexcept
    : BasicTopology(product(extents), product(extents), bandwidth, latency),
      x_extent(extents[0]),
      y_extent(extents[1]),
      z_extent(extents[2]),
      wraparound(wraparound) {
    validate_extents(extents, wraparound);

    if (topology_aware) {
        basic_topology_type = wraparound ? TopologyBuildingBlock::Torus3D3D
                                         : TopologyBuildingBlock::Mesh3D3D;
        dims_count = 3;
        npus_count_per_dim = {x_extent, y_extent, z_extent};
        bandwidth_per_dim = {bandwidth, bandwidth, bandwidth};
    } else {
        basic_topology_type = wraparound ? TopologyBuildingBlock::Torus3D
                                         : TopologyBuildingBlock::Mesh3D;
    }

    for (auto z = 0; z < z_extent; ++z) {
        for (auto y = 0; y < y_extent; ++y) {
            for (auto x = 0; x + 1 < x_extent; ++x) {
                connect(id_of(x, y, z), id_of(x + 1, y, z), bandwidth, latency,
                        true, LinkClass::BaseMesh);
            }
            if (wraparound) {
                connect(id_of(x_extent - 1, y, z), id_of(0, y, z), bandwidth,
                        latency, true, LinkClass::BaseMesh);
            }
        }
    }
    for (auto z = 0; z < z_extent; ++z) {
        for (auto x = 0; x < x_extent; ++x) {
            for (auto y = 0; y + 1 < y_extent; ++y) {
                connect(id_of(x, y, z), id_of(x, y + 1, z), bandwidth, latency,
                        true, LinkClass::BaseMesh);
            }
            if (wraparound) {
                connect(id_of(x, y_extent - 1, z), id_of(x, 0, z), bandwidth,
                        latency, true, LinkClass::BaseMesh);
            }
        }
    }
    for (auto y = 0; y < y_extent; ++y) {
        for (auto x = 0; x < x_extent; ++x) {
            for (auto z = 0; z + 1 < z_extent; ++z) {
                connect(id_of(x, y, z), id_of(x, y, z + 1), bandwidth, latency,
                        true, LinkClass::BaseMesh);
            }
            if (wraparound) {
                connect(id_of(x, y, z_extent - 1), id_of(x, y, 0), bandwidth,
                        latency, true, LinkClass::BaseMesh);
            }
        }
    }
}

int Mesh3D::x_of(const DeviceId id) const noexcept {
    return id % x_extent;
}

int Mesh3D::y_of(const DeviceId id) const noexcept {
    return (id / x_extent) % y_extent;
}

int Mesh3D::z_of(const DeviceId id) const noexcept {
    return id / (x_extent * y_extent);
}

DeviceId Mesh3D::id_of(const int x, const int y, const int z) const noexcept {
    return (z * y_extent + y) * x_extent + x;
}

int Mesh3D::step_towards(const int current, const int target,
                         const int extent,
                         const bool tie_backward) const noexcept {
    if (!wraparound) {
        return target > current ? current + 1 : current - 1;
    }
    const auto forward = (target - current + extent) % extent;
    const auto backward = extent - forward;
    if (forward < backward) {
        return (current + 1) % extent;
    }
    if (backward < forward) {
        return (current - 1 + extent) % extent;
    }
    return tie_backward ? (current - 1 + extent) % extent
                        : (current + 1) % extent;
}

Route Mesh3D::route(const DeviceId src, const DeviceId dest) const noexcept {
    assert(0 <= src && src < npus_count);
    assert(0 <= dest && dest < npus_count);

    auto path = Route();
    auto x = x_of(src);
    auto y = y_of(src);
    auto z = z_of(src);
    const auto destination_x = x_of(dest);
    const auto destination_y = y_of(dest);
    const auto destination_z = z_of(dest);
    const auto x_tie_backward = (x & 1) != 0;
    const auto y_tie_backward = (y & 1) != 0;
    const auto z_tie_backward = (z & 1) != 0;
    path.emplace_back(devices[src]);

    while (x != destination_x) {
        x = step_towards(x, destination_x, x_extent, x_tie_backward);
        path.emplace_back(devices[id_of(x, y, z)]);
    }
    while (y != destination_y) {
        y = step_towards(y, destination_y, y_extent, y_tie_backward);
        path.emplace_back(devices[id_of(x, y, z)]);
    }
    while (z != destination_z) {
        z = step_towards(z, destination_z, z_extent, z_tie_backward);
        path.emplace_back(devices[id_of(x, y, z)]);
    }
    return path;
}
