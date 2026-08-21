/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Topology.h"
#include "congestion_aware/Link.h"
#include <cassert>
#include <ostream>

using namespace NetworkAnalyticalCongestionAware;

void Topology::set_event_queue(std::shared_ptr<EventQueue> event_queue) noexcept {
    assert(event_queue != nullptr);

    // pass the given event_queue to Link
    Link::set_event_queue(std::move(event_queue));
}

Topology::Topology() noexcept : npus_count(-1), devices_count(-1), dims_count(-1) {
    npus_count_per_dim = {};
}

Route Topology::route(const DeviceId src, const DeviceId dest,
                      const ChunkSize chunk_size) const noexcept {
    static_cast<void>(chunk_size);
    return route(src, dest);
}

int Topology::get_devices_count() const noexcept {
    assert(devices_count > 0);
    assert(npus_count > 0);
    assert(devices_count >= npus_count);

    return devices_count;
}

int Topology::get_npus_count() const noexcept {
    assert(devices_count > 0);
    assert(npus_count > 0);
    assert(devices_count >= npus_count);

    return npus_count;
}

int Topology::get_dims_count() const noexcept {
    assert(dims_count > 0);

    return dims_count;
}

std::vector<int> Topology::get_npus_count_per_dim() const noexcept {
    assert(npus_count_per_dim.size() == dims_count);

    return npus_count_per_dim;
}

std::vector<Bandwidth> Topology::get_bandwidth_per_dim() const noexcept {
    assert(bandwidth_per_dim.size() == dims_count);

    return bandwidth_per_dim;
}

std::vector<LinkMetrics> Topology::get_link_metrics() const noexcept {
    auto metrics = std::vector<LinkMetrics>();
    for (const auto& device : devices) {
        auto device_metrics = device->get_link_metrics();
        metrics.insert(metrics.end(), device_metrics.begin(), device_metrics.end());
    }
    return metrics;
}

void Topology::print_link_metrics(std::ostream& output) const {
    for (const auto& metric : get_link_metrics()) {
        output << "NETWORK_LINK class=" << link_class_name(metric.link_class)
               << " src=" << metric.source
               << " port=" << metric.port
               << " dest=" << metric.destination
               << " bytes=" << metric.bytes
               << " messages=" << metric.messages
               << " peak_outstanding_bytes=" << metric.peak_outstanding_bytes
               << " busy_ns=" << metric.busy_time << '\n';
    }
}

void Topology::send(std::unique_ptr<Chunk> chunk) noexcept {
    assert(chunk != nullptr);

    // get src npu node_id
    const auto src = chunk->current_device()->get_id();

    // assert src is valid
    assert(0 <= src && src < devices_count);

    // Reserve the full selected path so adaptive route costs include traffic
    // assigned to downstream links but not yet physically present there.
    chunk->reserve_route();

    // initiate transmission from src
    devices[src]->send(std::move(chunk));
}

std::pair<LinkId, LinkId> Topology::connect(const DeviceId src,
                                           const DeviceId dest,
                                           const Bandwidth bandwidth,
                                           const Latency latency,
                                           const bool bidirectional,
                                           const LinkClass link_class) noexcept {
    // assert the src and dest are valid
    assert(0 <= src && src < devices_count);
    assert(0 <= dest && dest < devices_count);

    // assert bandwidth and latency are valid
    assert(bandwidth > 0);
    assert(latency >= 0);

    // connect src -> dest
    const auto forward = devices[src]->connect(dest, bandwidth, latency, link_class);

    // if bidirectional, connect dest -> src
    auto reverse = AutomaticLink;
    if (bidirectional) {
        reverse = devices[dest]->connect(src, bandwidth, latency, link_class);
    }
    return {forward, reverse};
}

void Topology::instantiate_devices() noexcept {
    // instantiate all devices
    for (auto i = 0; i < devices_count; i++) {
        devices.push_back(std::make_shared<Device>(i));
    }
}
