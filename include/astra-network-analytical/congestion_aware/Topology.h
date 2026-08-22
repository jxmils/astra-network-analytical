/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "common/EventQueue.h"
#include "congestion_aware/Chunk.h"
#include "congestion_aware/Device.h"
#include <iosfwd>
#include <map>
#include <memory>
#include <utility>
#include <vector>

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/**
 * Topology abstracts a network topology.
 */
class Topology {
  public:
    /**
     * Set the event queue to be used by the topology.
     *
     * @param event_queue pointer to the event queue
     */
    static void set_event_queue(std::shared_ptr<EventQueue> event_queue) noexcept;

    /**
     * Constructor.
     */
    Topology() noexcept;

    /**
     * Construct the route from src to dest.
     * Route is a list of devices (pointers) that the chunk should traverse,
     * including the src and dest devices themselves.
     *
     * e.g., route(0, 3) = [0, 5, 7, 2, 3]
     *
     * @param src src NPU id
     * @param dest dest NPU id
     *
     * @return route from src NPU to dest NPU
     */
    [[nodiscard]] virtual Route route(DeviceId src, DeviceId dest) const noexcept = 0;

    /**
     * Size-aware route selection. Existing topologies retain their fixed route;
     * adaptive topologies override this method.
     */
    [[nodiscard]] virtual Route route(DeviceId src, DeviceId dest,
                                      ChunkSize chunk_size) const noexcept;

    /** Request-aware selection for planned routes; fixed topologies ignore stream. */
    [[nodiscard]] virtual Route route(DeviceId src, DeviceId dest,
                                      ChunkSize chunk_size,
                                      int stream) const noexcept;

    /**
     * Initiate a transmission of a chunk.
     *
     * @param chunk chunk to be transmitted
     */
    virtual void send(std::unique_ptr<Chunk> chunk) noexcept;

    /**
     * Get the number of NPUs in the topology.
     * NPU excludes non-NPU devices such as switches.
     *
     * @return number of NPUs in the topology
     */
    [[nodiscard]] int get_npus_count() const noexcept;

    /**
     * Get the number of devices in the topology.
     * Device includes non-NPU devices such as switches.
     *
     * @return number of devices in the topology
     */
    [[nodiscard]] int get_devices_count() const noexcept;

    /**
     * Get the number of network dimensions.
     *
     * @return number of network dimensions
     */
    [[nodiscard]] int get_dims_count() const noexcept;

    /**
     * Get the number of NPUs per each dimension.
     *
     * @return number of NPUs per each dimension
     */
    [[nodiscard]] std::vector<int> get_npus_count_per_dim() const noexcept;

    /**
     * Get the bandwidth per each network dimension.
     *
     * @return bandwidth per each dimension
     */
    [[nodiscard]] std::vector<Bandwidth> get_bandwidth_per_dim() const noexcept;

    /** Retained per-port counters for audit and utilization reporting. */
    [[nodiscard]] virtual std::vector<LinkMetrics> get_link_metrics() const noexcept;
    virtual void print_link_metrics(std::ostream& output) const;

    /** Retained route selections grouped by route class and physical hops. */
    [[nodiscard]] std::vector<RouteMetrics> get_route_metrics() const noexcept;
    void print_route_metrics(std::ostream& output) const;

  protected:
    /// number of total devices in the topology
    /// device includes non-NPU devices such as switches
    int devices_count;

    /// number of NPUs in the topology
    /// NPU excludes non-NPU devices such as switches
    int npus_count;

    /// number of network dimensions
    int dims_count;

    /// number of NPUs per each dimension
    std::vector<int> npus_count_per_dim;

    /// holds the entire device instances in the topology
    std::vector<std::shared_ptr<Device>> devices;

    /// bandwidth per each network dimension
    std::vector<Bandwidth> bandwidth_per_dim;

    std::map<std::pair<RouteClass, int>, RouteMetrics> route_metrics;

    /// Shared simulator queue used by links and topology-level fabrics.
    static std::shared_ptr<EventQueue> event_queue;

    /**
     * Instantiate Device objects in the topology.
     */
    void instantiate_devices() noexcept;

    void record_route(const Route& route, ChunkSize chunk_size) noexcept;

    /**
     * Connect src -> dest with the given bandwidth and latency.
     * (i.e., a `Link` gets constructed between the two npus)
     *
     * if bidirectional=true, dest -> src connection is also established.
     *
     * @param src src device id
     * @param dest dest device id
     * @param bandwidth bandwidth of link
     * @param latency latency of link
     * @param bidirectional true if connection is bidirectional, false otherwise
     */
    std::pair<LinkId, LinkId> connect(
        DeviceId src, DeviceId dest, Bandwidth bandwidth, Latency latency,
        bool bidirectional = true,
        LinkClass link_class = LinkClass::Generic) noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
