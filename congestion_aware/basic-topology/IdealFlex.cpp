/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/IdealFlex.h"
#include "common/NetworkFunction.h"
#include <algorithm>
#include <cassert>
#include <cmath>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

namespace {

EventTime event_delay(const double delay) noexcept {
    return std::max<EventTime>(1, static_cast<EventTime>(std::ceil(delay)));
}

}  // namespace

IdealFlex::IdealFlex(const int npus_count, const Bandwidth bandwidth,
                     const Latency latency) noexcept
    : BasicTopology(npus_count, npus_count + Ports, bandwidth, latency),
      transmit_ready(npus_count, 0), receive_ready(npus_count, 0),
      batch_scheduled(false) {
    assert(npus_count == 64);
    basic_topology_type = TopologyBuildingBlock::IdealFlex6R4;
    dims_count = 3;
    npus_count_per_dim = {4, 4, 4};
    bandwidth_per_dim = {bandwidth, bandwidth, bandwidth};
    transmit_ports.assign(Ports, std::vector<LinkId>(npus_count));
    receive_ports.assign(Ports, std::vector<LinkId>(npus_count));
    for (auto port = 0; port < Ports; ++port) {
        const auto switch_id = npus_count + port;
        for (auto endpoint = 0; endpoint < npus_count; ++endpoint) {
            const auto links = connect(endpoint, switch_id, bandwidth, latency,
                                       true, LinkClass::SwitchUplink);
            transmit_ports[port][endpoint] = links.first;
            receive_ports[port][endpoint] = links.second;
            physical_metrics[{endpoint, links.first}] = {
                endpoint, switch_id, links.first, LinkClass::SwitchUplink,
                0, 0, 0, 0, 0};
            physical_metrics[{switch_id, links.second}] = {
                switch_id, endpoint, links.second, LinkClass::SwitchUplink,
                0, 0, 0, 0, 0};
        }
    }
}

Route IdealFlex::route(const DeviceId src, const DeviceId dest) const noexcept {
    assert(0 <= src && src < npus_count && 0 <= dest && dest < npus_count);
    if (src == dest) {
        return Route({devices[src]});
    }
    return Route({RouteHop(devices[src], transmit_ports[0][src]),
                  RouteHop(devices[npus_count], receive_ports[0][dest]),
                  RouteHop(devices[dest])});
}

void IdealFlex::send(std::unique_ptr<Chunk> chunk) noexcept {
    assert(chunk != nullptr);
    const auto& path = chunk->get_route();
    if (path.size() == 1) {
        chunk->invoke_callback();
        return;
    }
    const auto now = event_queue->get_current_time();
    pending.push_back(PendingTransfer{std::move(chunk), now});
    if (!batch_scheduled) {
        batch_scheduled = true;
        event_queue->schedule_event(now + 1, batch_callback, this);
    }
}

void IdealFlex::batch_callback(void* argument) noexcept {
    static_cast<IdealFlex*>(argument)->start_batch();
}

void IdealFlex::start_batch() noexcept {
    assert(batch_scheduled && !pending.empty());
    batch_scheduled = false;
    auto transfers = std::move(pending);
    pending.clear();
    auto groups = std::map<int, std::vector<PendingTransfer>>();
    for (auto& transfer : transfers) {
        groups[transfer.chunk->get_stream()].push_back(std::move(transfer));
    }
    for (auto& [stream, group] : groups) {
        static_cast<void>(stream);
        auto source_bytes = std::vector<ChunkSize>(npus_count, 0);
        auto destination_bytes = std::vector<ChunkSize>(npus_count, 0);
        auto start = event_queue->get_current_time();
        for (const auto& transfer : group) {
            const auto& path = transfer.chunk->get_route();
            const auto source = path.front().device->get_id();
            const auto destination = path.back().device->get_id();
            source_bytes[source] += transfer.chunk->get_size();
            destination_bytes[destination] += transfer.chunk->get_size();
            start = std::max({start, transmit_ready[source],
                              receive_ready[destination]});
        }
        const auto maximum_load = std::max(
            *std::max_element(source_bytes.begin(), source_bytes.end()),
            *std::max_element(destination_bytes.begin(), destination_bytes.end()));
        const auto serialization = event_delay(
            static_cast<double>(maximum_load) /
            bw_GBps_to_Bpns(Ports * bandwidth));
        const auto finish = start + serialization;
        for (auto endpoint = 0; endpoint < npus_count; ++endpoint) {
            if (source_bytes[endpoint]) {
                transmit_ready[endpoint] = finish;
            }
            if (destination_bytes[endpoint]) {
                receive_ready[endpoint] = finish;
            }
        }

        auto chunks = std::vector<std::unique_ptr<Chunk>>();
        chunks.reserve(group.size());
        for (auto& transfer : group) {
            const auto& path = transfer.chunk->get_route();
            const auto source = path.front().device->get_id();
            const auto destination = path.back().device->get_id();
            const auto bytes = transfer.chunk->get_size();
            record_route_metrics(
                RouteClass::Switch, 1, 2, bytes, event_delay(2.0 * latency),
                event_delay(static_cast<double>(bytes) /
                            bw_GBps_to_Bpns(Ports * bandwidth)));
            const auto base = bytes / Ports;
            const auto remainder = bytes % Ports;
            for (auto port = 0; port < Ports; ++port) {
                const auto stripe = base + static_cast<ChunkSize>(port < remainder);
                if (stripe == 0) {
                    continue;
                }
                const auto stripe_busy = event_delay(
                    static_cast<double>(stripe) / bw_GBps_to_Bpns(bandwidth));
                const auto switch_id = npus_count + port;
                for (const auto key : {
                         std::make_pair(source, transmit_ports[port][source]),
                         std::make_pair(switch_id, receive_ports[port][destination])}) {
                    auto& metric = physical_metrics.at(key);
                    metric.bytes += stripe;
                    ++metric.messages;
                    metric.peak_outstanding_bytes = std::max(
                        metric.peak_outstanding_bytes,
                        static_cast<uint64_t>(stripe));
                    metric.busy_time += stripe_busy;
                    metric.queue_wait_time += start - transfer.queued_at;
                }
            }
            chunks.push_back(std::move(transfer.chunk));
        }
        auto* completion = new Completion{std::move(chunks)};
        event_queue->schedule_event(finish + event_delay(2.0 * latency),
                                    completion_callback, completion);
    }
}

void IdealFlex::completion_callback(void* argument) noexcept {
    auto completion = std::unique_ptr<Completion>(
        static_cast<Completion*>(argument));
    for (auto& chunk : completion->chunks) {
        chunk->invoke_callback();
    }
}

std::vector<LinkMetrics> IdealFlex::get_link_metrics() const noexcept {
    auto result = Topology::get_link_metrics();
    for (auto& metric : result) {
        const auto found = physical_metrics.find({metric.source, metric.port});
        if (found != physical_metrics.end()) {
            metric = found->second;
        }
    }
    return result;
}
