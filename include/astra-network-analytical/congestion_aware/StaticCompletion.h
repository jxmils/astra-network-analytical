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

/** A 2-D torus completed by fixed perfect matchings on optical planes. */
class StaticCompletion final : public BasicTopology {
  public:
    StaticCompletion(int npus_count, Bandwidth bandwidth, Latency latency,
                     Bandwidth optical_bandwidth, Latency optical_leg_latency,
                     const std::string& plan_path) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest,
                              ChunkSize chunk_size) const noexcept override;

  private:
    struct Arc {
        DeviceId destination;
        int plane;
        LinkId source_port;
        LinkId switch_port;
    };

    int side;
    int planes;
    Bandwidth optical_bandwidth;
    Latency optical_leg_latency;
    std::vector<std::vector<Arc>> adjacency;

    void build_base_torus() noexcept;
    void load_matchings(const std::string& path) noexcept;
    [[nodiscard]] double arc_cost(DeviceId source, const Arc& arc,
                                  ChunkSize bytes) const noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
