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
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace NetworkAnalyticalCongestionAware {

class OcsSwitch final : public BasicTopology {
  public:
    OcsSwitch(int npus_count, Bandwidth bandwidth, Latency latency,
              const std::string& plan_path, int expected_planes = 6,
              bool base_torus = false, bool qtp_embedding = false,
              bool base_ring = false) noexcept;

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
    [[nodiscard]] uint64_t get_escaped_bytes() const noexcept;
    [[nodiscard]] uint64_t get_escaped_assignments() const noexcept;
    [[nodiscard]] EventTime get_reconfiguration_time() const noexcept;
    [[nodiscard]] EventTime get_max_plane_reconfiguration_time() const noexcept;
    [[nodiscard]] EventTime get_critical_plane_reconfiguration_time() const noexcept;
    [[nodiscard]] EventTime get_plane_schedule_makespan() const noexcept;
    [[nodiscard]] EventTime get_circuit_wait_time() const noexcept;
    [[nodiscard]] EventTime get_max_circuit_wait_time() const noexcept;
    [[nodiscard]] uint64_t get_circuit_transmissions() const noexcept;
    [[nodiscard]] int get_max_active_ports(DeviceId endpoint) const noexcept;
    [[nodiscard]] int get_max_distinct_peers(DeviceId endpoint) const noexcept;
    [[nodiscard]] const std::vector<int>& get_logical_to_physical() const noexcept;

  private:
    using Pair = std::pair<DeviceId, DeviceId>;
    using AssignmentKey = std::tuple<DeviceId, DeviceId, ChunkSize, int>;
    struct Circuit {
        Pair pair;
        uint64_t bytes;
        uint64_t remaining;
        bool busy;
        ChunkSize busy_bytes;
    };

    struct Configuration {
        int plane;
        int stream;
        bool force_reconfiguration;
        int round;
        bool synchronize;
        std::vector<Pair> matching;
        std::vector<Circuit> circuits;
    };

    struct PlaneState {
        std::vector<Configuration> configurations;
        std::size_t current = 0;
        bool reconfiguring = false;
        bool activated = false;
        bool has_installed_matching = false;
        int completed = 0;
        int reconfigurations = 0;
        EventTime reconfiguration_time = 0;
        EventTime data_time = 0;
        std::vector<Pair> installed_matching;
    };

    struct PlaneStripe {
        int plane;
        ChunkSize bytes;
    };

    struct RuntimeAssignment {
        bool direct;
        std::vector<PlaneStripe> stripes;
        EventTime not_before;
        int request_id;
        bool allow_direct_escape;
        int target_round;
    };

    struct LogicalTransfer {
        std::unique_ptr<Chunk> chunk;
        Pair pair;
        int stream;
        EventTime queued_at;
        int remaining_stripes;
    };

    struct PendingStripe {
        std::shared_ptr<LogicalTransfer> transfer;
        ChunkSize bytes;
    };

    struct Transmission {
        OcsSwitch* topology;
        std::shared_ptr<LogicalTransfer> transfer;
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
    bool qtp_embedding;
    bool base_ring;
    int width;
    double plan_bandwidth;
    double propagation_ns;
    double reconfiguration_ns;
    double direct_escape_factor;
    bool initial_reconfiguration;
    std::vector<PlaneState> plane_states;
    std::vector<std::vector<LinkId>> to_switch_ports;
    std::vector<std::vector<LinkId>> from_switch_ports;
    std::map<Pair, LinkId> base_ports;
    std::vector<int> logical_to_physical;
    std::vector<int> physical_to_logical;
    std::vector<int> plan_dimensions;
    std::map<std::tuple<int, DeviceId, DeviceId, int>,
             std::deque<PendingStripe>> pending;
    mutable std::map<AssignmentKey, std::deque<RuntimeAssignment>> route_assignments;
    mutable std::map<AssignmentKey, std::deque<RuntimeAssignment>> dispatch_assignments;
    std::map<std::pair<DeviceId, LinkId>, LinkMetrics> physical_metrics;
    bool epoch_started;
    EventTime epoch_start;
    int reconfiguration_count;
    uint64_t scheduled_bytes;
    uint64_t transmitted_bytes;
    mutable uint64_t escaped_bytes;
    mutable uint64_t escaped_assignments;
    EventTime circuit_wait_time;
    EventTime max_circuit_wait_time;
    uint64_t circuit_transmissions;
    uint64_t planned_assignments;
    mutable uint64_t consumed_assignments;
    uint64_t causal_dispatches;
    EventTime max_release_slip;
    int max_release_slip_request;
    EventTime max_release_slip_planned;
    EventTime max_release_slip_actual;
    std::vector<std::tuple<int, EventTime, EventTime>> release_records;
    std::vector<std::map<int, int>> active_endpoint_planes;
    std::vector<std::map<int, int>> active_endpoint_tx_planes;
    std::vector<std::map<int, int>> active_endpoint_rx_planes;
    std::vector<std::map<DeviceId, int>> active_endpoint_peers;
    std::vector<std::map<DeviceId, int>> active_endpoint_tx_peers;
    std::vector<std::map<DeviceId, int>> active_endpoint_rx_peers;
    std::vector<int> max_active_endpoint_ports;
    std::vector<int> max_active_endpoint_tx_ports;
    std::vector<int> max_active_endpoint_rx_ports;
    std::vector<int> max_distinct_endpoint_peers;
    std::vector<int> max_distinct_endpoint_tx_peers;
    std::vector<int> max_distinct_endpoint_rx_peers;

    void load_plan(const std::string& path) noexcept;
    void validate_plan() const noexcept;
    void build_qtp_embedding() noexcept;
    void build_base_torus() noexcept;
    void build_base_ring() noexcept;
    [[nodiscard]] Route direct_route(DeviceId src, DeviceId dest) const noexcept;
    [[nodiscard]] int step_towards(int current, int target, int extent,
                                   bool tie_backward) const noexcept;
    void start_initial_planes() noexcept;
    void activate_configuration(int plane) noexcept;
    void try_activate_synchronized_round(int round) noexcept;
    void dispatch(std::unique_ptr<Chunk> chunk,
                  const RuntimeAssignment& assignment) noexcept;
    [[nodiscard]] Route ocs_route(DeviceId src, DeviceId dest,
                                  int plane) const noexcept;
    [[nodiscard]] double direct_path_cost(DeviceId src, DeviceId dest,
                                          ChunkSize bytes) const noexcept;
    [[nodiscard]] double optical_path_cost(DeviceId src, DeviceId dest,
                                           ChunkSize bytes,
                                           const RuntimeAssignment& assignment) const noexcept;
    [[nodiscard]] bool should_escape_direct(DeviceId src, DeviceId dest,
                                            ChunkSize bytes,
                                            const RuntimeAssignment& assignment) const noexcept;
    void consume_escaped_quota(DeviceId src, DeviceId dest,
                               const RuntimeAssignment& assignment) noexcept;
    void try_start_transmissions(int plane) noexcept;
    void start_transmission(int plane, Circuit& circuit,
                            PendingStripe stripe) noexcept;
    void finish_serialization(Transmission* transmission) noexcept;
    void finish_arrival(Transmission* transmission) noexcept;
    void advance_configuration(int plane) noexcept;
    [[nodiscard]] bool configuration_complete(int plane) const noexcept;
    [[nodiscard]] bool matching_changed(const std::vector<Pair>& first,
                                        const std::vector<Pair>& second) const noexcept;
    [[nodiscard]] Circuit* find_circuit(int plane, const Pair& pair) noexcept;

    static void serialization_callback(void* argument) noexcept;
    static void arrival_callback(void* argument) noexcept;
    static void reconfiguration_callback(void* argument) noexcept;
    static void delayed_chunk_callback(void* argument) noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
