/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"
#include <map>
#include <memory>
#include <vector>

namespace NetworkAnalyticalCongestionAware {

/** Dependency-preserving fluid service over six perfectly flexible ports. */
class IdealFlex final : public BasicTopology {
  public:
    IdealFlex(int npus_count, Bandwidth bandwidth, Latency latency) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
    void send(std::unique_ptr<Chunk> chunk) noexcept override;
    [[nodiscard]] std::vector<LinkMetrics> get_link_metrics() const noexcept override;

  private:
    struct Completion {
        std::vector<std::unique_ptr<Chunk>> chunks;
    };

    struct PendingTransfer {
        std::unique_ptr<Chunk> chunk;
        EventTime queued_at;
    };

    static constexpr int Ports = 6;
    std::vector<EventTime> transmit_ready;
    std::vector<EventTime> receive_ready;
    std::vector<PendingTransfer> pending;
    bool batch_scheduled;
    std::vector<std::vector<LinkId>> transmit_ports;
    std::vector<std::vector<LinkId>> receive_ports;
    std::map<std::pair<DeviceId, LinkId>, LinkMetrics> physical_metrics;

    static void completion_callback(void* argument) noexcept;
    static void batch_callback(void* argument) noexcept;
    void start_batch() noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
