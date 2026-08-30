/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"
#include <string>
#include <vector>

namespace NetworkAnalyticalCongestionAware {

/** A congestion-aware physical graph loaded from a versioned YAML file. */
class FileGraph final : public BasicTopology {
  public:
    FileGraph(int npus_count, Bandwidth bandwidth, Latency latency,
              const std::string& graph_path) noexcept;

    [[nodiscard]] Route route(DeviceId src,
                              DeviceId dest) const noexcept override;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest,
                              ChunkSize chunk_size) const noexcept override;

  private:
    struct Arc {
        DeviceId destination;
        LinkId source_port;
    };

    std::vector<std::vector<Arc>> adjacency;

    [[nodiscard]] double arc_cost(DeviceId source, const Arc& arc,
                                  ChunkSize bytes) const noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
