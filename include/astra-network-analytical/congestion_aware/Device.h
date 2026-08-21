/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "common/Type.h"
#include "congestion_aware/Type.h"
#include <map>
#include <memory>
#include <utility>
#include <vector>

using namespace NetworkAnalytical;

namespace NetworkAnalyticalCongestionAware {

/**
 * Device class represents a single device in the network.
 * Device is usually an NPU or a switch.
 */
class Device {
  public:
    /**
     * Constructor.
     *
     * @param id id of the device
     */
    explicit Device(DeviceId id) noexcept;

    /**
     * Get id of the device.
     *
     * @return id of the device
     */
    [[nodiscard]] DeviceId get_id() const noexcept;

    /**
     * Return the destination IDs of this device's outgoing links.
     *
     * This is topology metadata only; callers cannot mutate links through it.
     */
    [[nodiscard]] std::vector<DeviceId> get_connected_device_ids() const noexcept;

    /** Return metadata for every outgoing physical port. */
    [[nodiscard]] std::vector<LinkMetrics> get_link_metrics() const noexcept;

    /** Return the class of one outgoing physical port. */
    [[nodiscard]] LinkClass get_link_class(LinkId link_id) const noexcept;

    /** Return whether a port connects to the requested next device. */
    [[nodiscard]] bool link_connects(LinkId link_id, DeviceId destination) const noexcept;

    /** Estimated outstanding bytes already assigned to a port. */
    [[nodiscard]] uint64_t get_outstanding_bytes(LinkId link_id) const noexcept;

    /** Port bandwidth and latency used by path-cost calculations. */
    [[nodiscard]] Bandwidth get_link_bandwidth(LinkId link_id) const noexcept;
    [[nodiscard]] Latency get_link_latency(LinkId link_id) const noexcept;

    /** Resolve an exact or legacy automatic port for one route hop. */
    [[nodiscard]] LinkId resolve_link(LinkId link_id,
                                      DeviceId destination) const noexcept;

    /**
     * Initiate a chunk transmission.
     * You must invoke this method on the source device of the chunk.
     *
     * @param chunk chunk to send
     */
    void send(std::unique_ptr<Chunk> chunk) noexcept;

    /**
     * Connect a device to another device.
     *
     * @param id id of the device to connect this device to
     * @param bandwidth bandwidth of the link
     * @param latency latency of the link
     */
    LinkId connect(DeviceId id, Bandwidth bandwidth, Latency latency,
                   LinkClass link_class = LinkClass::Generic) noexcept;

    /** Reserve all bytes assigned to a port before transmission begins. */
    void reserve(LinkId link_id, DeviceId destination, ChunkSize chunk_size) noexcept;

  private:
    /// device Id
    DeviceId device_id;

    struct Connection {
        DeviceId destination;
        std::shared_ptr<Link> link;
    };

    /// Local physical ports. Multiple entries may have the same destination.
    std::map<LinkId, Connection> links;

    /// Next local physical-port ID.
    LinkId next_link_id;

    /** Resolve an old-style route hop when exactly one port reaches dest. */
    [[nodiscard]] LinkId unique_link_to(DeviceId dest) const noexcept;
};

}  // namespace NetworkAnalyticalCongestionAware
