/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"
#include <string>
#include <vector>

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/** A persistent base fabric completed by fixed optical perfect matchings. */
class StaticCompletion final : public BasicTopology {
  public:
    enum class BaseFabric { Torus2D, RowRings };

    StaticCompletion(int npus_count, Bandwidth bandwidth, Latency latency,
                     Bandwidth optical_bandwidth, Latency optical_path_latency,
                     const std::string& plan_path,
                     BaseFabric base_fabric = BaseFabric::Torus2D) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest,
                              ChunkSize chunk_size) const noexcept override;

  private:
    struct Arc {
        DeviceId destination;
        LinkId source_port;
    };

    int side;
    int planes;
    Bandwidth optical_bandwidth;
    Latency optical_path_latency;
    BaseFabric base_fabric;
    std::vector<std::vector<Arc>> adjacency;

    void build_base() noexcept;
    void load_matchings(const std::string& path) noexcept;
    [[nodiscard]] double arc_cost(DeviceId source, const Arc& arc,
                                  ChunkSize bytes) const noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
