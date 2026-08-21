/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Mesh3D.h"
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

}  // namespace

Mesh3D::Mesh3D(const int npus_count, const Bandwidth bandwidth,
               const Latency latency, const bool wraparound) noexcept
    : BasicTopology(npus_count, npus_count, bandwidth, latency),
      extent(static_cast<int>(std::lround(std::cbrt(
          static_cast<double>(npus_count))))),
      wraparound(wraparound) {
    if (extent * extent * extent != npus_count) {
        reject_mesh3d_configuration("Mesh3D/Torus3D requires a perfect-cube npus_count");
    }
    if (wraparound && extent <= 2) {
        reject_mesh3d_configuration(
            "Torus3D requires extent greater than two for distinct neighbors");
    }

    basic_topology_type = wraparound ? TopologyBuildingBlock::Torus3D
                                     : TopologyBuildingBlock::Mesh3D;

    for (auto z = 0; z < extent; ++z) {
        for (auto y = 0; y < extent; ++y) {
            for (auto x = 0; x + 1 < extent; ++x) {
                connect(id_of(x, y, z), id_of(x + 1, y, z), bandwidth, latency,
                        true, LinkClass::BaseMesh);
            }
            if (wraparound) {
                connect(id_of(extent - 1, y, z), id_of(0, y, z), bandwidth,
                        latency, true, LinkClass::BaseMesh);
            }
        }
    }
    for (auto z = 0; z < extent; ++z) {
        for (auto x = 0; x < extent; ++x) {
            for (auto y = 0; y + 1 < extent; ++y) {
                connect(id_of(x, y, z), id_of(x, y + 1, z), bandwidth, latency,
                        true, LinkClass::BaseMesh);
            }
            if (wraparound) {
                connect(id_of(x, extent - 1, z), id_of(x, 0, z), bandwidth,
                        latency, true, LinkClass::BaseMesh);
            }
        }
    }
    for (auto y = 0; y < extent; ++y) {
        for (auto x = 0; x < extent; ++x) {
            for (auto z = 0; z + 1 < extent; ++z) {
                connect(id_of(x, y, z), id_of(x, y, z + 1), bandwidth, latency,
                        true, LinkClass::BaseMesh);
            }
            if (wraparound) {
                connect(id_of(x, y, extent - 1), id_of(x, y, 0), bandwidth,
                        latency, true, LinkClass::BaseMesh);
            }
        }
    }
}

int Mesh3D::x_of(const DeviceId id) const noexcept {
    return id % extent;
}

int Mesh3D::y_of(const DeviceId id) const noexcept {
    return (id / extent) % extent;
}

int Mesh3D::z_of(const DeviceId id) const noexcept {
    return id / (extent * extent);
}

DeviceId Mesh3D::id_of(const int x, const int y, const int z) const noexcept {
    return (z * extent + y) * extent + x;
}

int Mesh3D::step_towards(const int current, const int target,
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
        x = step_towards(x, destination_x, x_tie_backward);
        path.emplace_back(devices[id_of(x, y, z)]);
    }
    while (y != destination_y) {
        y = step_towards(y, destination_y, y_tie_backward);
        path.emplace_back(devices[id_of(x, y, z)]);
    }
    while (z != destination_z) {
        z = step_towards(z, destination_z, z_tie_backward);
        path.emplace_back(devices[id_of(x, y, z)]);
    }
    return path;
}
