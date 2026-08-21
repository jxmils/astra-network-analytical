/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Link.h"
#include "common/NetworkFunction.h"
#include "congestion_aware/Chunk.h"
#include "congestion_aware/Device.h"
#include <algorithm>
#include <cassert>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

// declaring static event_queue
std::shared_ptr<EventQueue> Link::event_queue;

void Link::link_become_free(void* const link_ptr) noexcept {
    assert(link_ptr != nullptr);

    // cast to Link*
    auto* const link = static_cast<Link*>(link_ptr);

    assert(link->active_chunk_size > 0);
    assert(link->outstanding_bytes >= link->active_chunk_size);
    link->outstanding_bytes -= link->active_chunk_size;
    link->active_chunk_size = 0;

    // set link free
    link->set_free();

    // process pending chunks if one exist
    if (link->pending_chunk_exists()) {
        link->process_pending_transmission();
    }
}

void Link::set_event_queue(std::shared_ptr<EventQueue> event_queue_ptr) noexcept {
    assert(event_queue_ptr != nullptr);

    // set the event queue
    Link::event_queue = std::move(event_queue_ptr);
}

Link::Link(const Bandwidth bandwidth, const Latency latency, const LinkClass link_class) noexcept
    : bandwidth(bandwidth),
      latency(latency),
      pending_chunks(),
      busy(false),
      link_class(link_class),
      outstanding_bytes(0),
      peak_outstanding_bytes(0),
      transmitted_bytes(0),
      transmitted_messages(0),
      busy_time(0),
      queue_wait_time(0),
      active_chunk_size(0) {
    assert(bandwidth > 0);
    assert(latency >= 0);

    // convert bandwidth from GB/s to B/ns
    bandwidth_Bpns = bw_GBps_to_Bpns(bandwidth);
}

void Link::send(std::unique_ptr<Chunk> chunk) noexcept {
    assert(chunk != nullptr);

    chunk->mark_link_queued(event_queue->get_current_time());

    if (busy) {
        // link is busy, add to pending chunks
        pending_chunks.push_back(std::move(chunk));
    } else {
        // service this chunk immediately
        schedule_chunk_transmission(std::move(chunk));
    }
}

void Link::process_pending_transmission() noexcept {
    // pending chunk should exist
    assert(pending_chunk_exists());

    // get chunk to process
    auto chunk = std::move(pending_chunks.front());
    pending_chunks.pop_front();

    // service this chunk
    schedule_chunk_transmission(std::move(chunk));
}

bool Link::pending_chunk_exists() const noexcept {
    // check pending chunks is not empty
    return !pending_chunks.empty();
}

void Link::set_busy() noexcept {
    // set busy to true
    busy = true;
}

void Link::set_free() noexcept {
    // set busy to false
    busy = false;
}

void Link::reserve(const ChunkSize chunk_size) noexcept {
    assert(chunk_size > 0);
    outstanding_bytes += chunk_size;
    peak_outstanding_bytes = std::max(peak_outstanding_bytes, outstanding_bytes);
}

uint64_t Link::get_outstanding_bytes() const noexcept {
    return outstanding_bytes;
}

Bandwidth Link::get_bandwidth() const noexcept {
    return bandwidth;
}

Latency Link::get_latency() const noexcept {
    return latency;
}

LinkClass Link::get_link_class() const noexcept {
    return link_class;
}

uint64_t Link::get_transmitted_bytes() const noexcept {
    return transmitted_bytes;
}

uint64_t Link::get_transmitted_messages() const noexcept {
    return transmitted_messages;
}

uint64_t Link::get_peak_outstanding_bytes() const noexcept {
    return peak_outstanding_bytes;
}

EventTime Link::get_busy_time() const noexcept {
    return busy_time;
}

EventTime Link::get_queue_wait_time() const noexcept {
    return queue_wait_time;
}

EventTime Link::serialization_delay(const ChunkSize chunk_size) const noexcept {
    assert(chunk_size > 0);

    // calculate serialization delay
    const auto delay = static_cast<Bandwidth>(chunk_size) / bandwidth_Bpns;

    // return serialization delay in EventTime type
    return static_cast<EventTime>(delay);
}

EventTime Link::communication_delay(const ChunkSize chunk_size) const noexcept {
    assert(chunk_size > 0);

    // calculate communication delay
    const auto delay = latency + (static_cast<Bandwidth>(chunk_size) / bandwidth_Bpns);

    // return communication delay in EventTime type
    return static_cast<EventTime>(delay);
}

void Link::schedule_chunk_transmission(std::unique_ptr<Chunk> chunk) noexcept {
    assert(chunk != nullptr);

    // link should be free
    assert(!busy);

    // set link busy
    set_busy();

    // get metadata
    const auto chunk_size = chunk->get_size();
    const auto current_time = Link::event_queue->get_current_time();
    const auto serialization_time = serialization_delay(chunk_size);

    assert(current_time >= chunk->get_link_queued_time());
    queue_wait_time += current_time - chunk->get_link_queued_time();

    assert(active_chunk_size == 0);
    assert(outstanding_bytes >= chunk_size);
    active_chunk_size = chunk_size;
    transmitted_bytes += chunk_size;
    transmitted_messages++;
    busy_time += serialization_time;

    // schedule chunk arrival event
    const auto communication_time = communication_delay(chunk_size);
    const auto chunk_arrival_time = current_time + communication_time;
    auto* const chunk_ptr = static_cast<void*>(chunk.release());
    Link::event_queue->schedule_event(chunk_arrival_time, Chunk::chunk_arrived_next_device, chunk_ptr);

    // schedule link free time
    const auto link_free_time = current_time + serialization_time;
    auto* const link_ptr = static_cast<void*>(this);
    Link::event_queue->schedule_event(link_free_time, link_become_free, link_ptr);
}
