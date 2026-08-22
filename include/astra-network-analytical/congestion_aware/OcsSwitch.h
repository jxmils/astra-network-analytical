/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "congestion_aware/BasicTopology.h"
#include <deque>
#include <map>
#include <memory>
#include <iosfwd>
#include <string>
#include <tuple>
#include <vector>

namespace NetworkAnalyticalCongestionAware {

class OcsSwitch final : public BasicTopology {
  public:
    OcsSwitch(int npus_count, Bandwidth bandwidth, Latency latency,
              const std::string& plan_path, int expected_planes = 6,
              bool base_torus = false) noexcept;

    [[nodiscard]] Route route(DeviceId src, DeviceId dest) const noexcept override;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest,
                              ChunkSize chunk_size) const noexcept override;
    [[nodiscard]] Route route(DeviceId src, DeviceId dest, ChunkSize chunk_size,
                              int stream) const noexcept override;
    void send(std::unique_ptr<Chunk> chunk) noexcept override;
    [[nodiscard]] std::vector<LinkMetrics> get_link_metrics() const noexcept override;
    void print_link_metrics(std::ostream& output) const override;

    [[nodiscard]] int get_plane_count() const noexcept;
    [[nodiscard]] int get_round_count() const noexcept;
    [[nodiscard]] int get_completed_rounds() const noexcept;
    [[nodiscard]] int get_reconfiguration_count() const noexcept;
    [[nodiscard]] uint64_t get_scheduled_bytes() const noexcept;
    [[nodiscard]] uint64_t get_transmitted_bytes() const noexcept;
    [[nodiscard]] EventTime get_reconfiguration_time() const noexcept;

  private:
    using Pair = std::pair<DeviceId, DeviceId>;
    using AssignmentKey = std::tuple<DeviceId, DeviceId, ChunkSize, int>;

    struct Circuit {
        Pair pair;
        uint64_t bytes;
        uint64_t remaining;
        bool busy;
    };

    struct Configuration {
        int plane;
        std::vector<Circuit> circuits;
    };

    struct Round {
        int index;
        std::vector<Configuration> configurations;
    };

    struct Transmission {
        OcsSwitch* topology;
        std::unique_ptr<Chunk> chunk;
        int plane;
        Pair pair;
        ChunkSize bytes;
    };

    int planes;
    int expected_planes;
    bool base_torus;
    int width;
    double plan_bandwidth;
    double propagation_ns;
    double reconfiguration_ns;
    bool initial_reconfiguration;
    std::vector<Round> rounds;
    std::vector<std::vector<LinkId>> to_switch_ports;
    std::vector<std::vector<LinkId>> from_switch_ports;
    std::map<Pair, LinkId> base_ports;
    std::map<Pair, std::deque<std::unique_ptr<Chunk>>> pending;
    mutable std::map<AssignmentKey, std::deque<bool>> route_assignments;
    std::map<std::pair<DeviceId, LinkId>, LinkMetrics> physical_metrics;
    int current_round;
    bool reconfiguring;
    int completed_rounds;
    int reconfiguration_count;
    uint64_t scheduled_bytes;
    uint64_t transmitted_bytes;
    uint64_t planned_assignments;
    mutable uint64_t consumed_assignments;
    EventTime reconfiguration_time;

    void load_plan(const std::string& path) noexcept;
    void validate_plan() const noexcept;
    void build_base_torus() noexcept;
    [[nodiscard]] Route direct_route(DeviceId src, DeviceId dest) const noexcept;
    [[nodiscard]] int step_towards(int current, int target, int extent,
                                   bool tie_backward) const noexcept;
    void start_initial_round() noexcept;
    void try_start_transmissions() noexcept;
    void start_transmission(int plane, Circuit& circuit,
                            std::unique_ptr<Chunk> chunk) noexcept;
    void finish_serialization(Transmission* transmission) noexcept;
    void finish_arrival(Transmission* transmission) noexcept;
    void advance_round() noexcept;
    [[nodiscard]] bool round_complete() const noexcept;
    [[nodiscard]] bool configuration_changed(const Round& first,
                                             const Round& second) const noexcept;
    [[nodiscard]] Circuit* find_circuit(int plane, const Pair& pair) noexcept;

    static void serialization_callback(void* argument) noexcept;
    static void arrival_callback(void* argument) noexcept;
    static void reconfiguration_callback(void* argument) noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
