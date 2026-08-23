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
        int stream;
        std::vector<Pair> matching;
        std::vector<Circuit> circuits;
    };

    struct PlaneState {
        std::vector<Configuration> configurations;
        std::size_t current = 0;
        bool reconfiguring = false;
        int completed = 0;
        int reconfigurations = 0;
        EventTime reconfiguration_time = 0;
    };

    struct RuntimeAssignment {
        int plane;
        EventTime not_before;
        int request_id;
    };

    struct Transmission {
        OcsSwitch* topology;
        std::unique_ptr<Chunk> chunk;
        int plane;
        Pair pair;
        ChunkSize bytes;
    };

    struct DelayedChunk {
        OcsSwitch* topology;
        std::unique_ptr<Chunk> chunk;
        RuntimeAssignment assignment;
    };

    struct PlaneCallback {
        OcsSwitch* topology;
        int plane;
    };

    int planes;
    int expected_planes;
    bool base_torus;
    int width;
    double plan_bandwidth;
    double propagation_ns;
    double reconfiguration_ns;
    bool initial_reconfiguration;
    std::vector<PlaneState> plane_states;
    std::vector<std::vector<LinkId>> to_switch_ports;
    std::vector<std::vector<LinkId>> from_switch_ports;
    std::map<Pair, LinkId> base_ports;
    std::map<std::tuple<int, DeviceId, DeviceId, int>,
             std::deque<std::unique_ptr<Chunk>>> pending;
    mutable std::map<AssignmentKey, std::deque<RuntimeAssignment>> route_assignments;
    mutable std::map<AssignmentKey, std::deque<RuntimeAssignment>> dispatch_assignments;
    std::map<std::pair<DeviceId, LinkId>, LinkMetrics> physical_metrics;
    bool epoch_started;
    EventTime epoch_start;
    int reconfiguration_count;
    uint64_t scheduled_bytes;
    uint64_t transmitted_bytes;
    uint64_t planned_assignments;
    mutable uint64_t consumed_assignments;
    uint64_t causal_dispatches;
    EventTime max_release_slip;

    void load_plan(const std::string& path) noexcept;
    void validate_plan() const noexcept;
    void build_base_torus() noexcept;
    [[nodiscard]] Route direct_route(DeviceId src, DeviceId dest) const noexcept;
    [[nodiscard]] int step_towards(int current, int target, int extent,
                                   bool tie_backward) const noexcept;
    void start_initial_planes() noexcept;
    void dispatch(std::unique_ptr<Chunk> chunk,
                  const RuntimeAssignment& assignment) noexcept;
    [[nodiscard]] Route ocs_route(DeviceId src, DeviceId dest,
                                  int plane) const noexcept;
    void try_start_transmissions(int plane) noexcept;
    void start_transmission(int plane, Circuit& circuit,
                            std::unique_ptr<Chunk> chunk) noexcept;
    void finish_serialization(Transmission* transmission) noexcept;
    void finish_arrival(Transmission* transmission) noexcept;
    void advance_configuration(int plane) noexcept;
    [[nodiscard]] bool configuration_complete(int plane) const noexcept;
    [[nodiscard]] bool configuration_changed(const Configuration& first,
                                             const Configuration& second) const noexcept;
    [[nodiscard]] Circuit* find_circuit(int plane, const Pair& pair) noexcept;

    static void serialization_callback(void* argument) noexcept;
    static void arrival_callback(void* argument) noexcept;
    static void reconfiguration_callback(void* argument) noexcept;
    static void delayed_chunk_callback(void* argument) noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
