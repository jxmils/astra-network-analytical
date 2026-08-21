/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Device.h"
#include "congestion_aware/Chunk.h"
#include "congestion_aware/Link.h"
#include <cassert>
#include <cstdlib>
#include <iostream>

using namespace NetworkAnalyticalCongestionAware;

Device::Device(const DeviceId id) noexcept : device_id(id), next_link_id(0) {
    assert(id >= 0);
}

DeviceId Device::get_id() const noexcept {
    assert(device_id >= 0);

    return device_id;
}

std::vector<DeviceId> Device::get_connected_device_ids() const noexcept {
    auto connected_device_ids = std::vector<DeviceId>();
    connected_device_ids.reserve(links.size());
    for (const auto& connection : links) {
        connected_device_ids.push_back(connection.second.destination);
    }
    return connected_device_ids;
}

std::vector<LinkMetrics> Device::get_link_metrics() const noexcept {
    auto metrics = std::vector<LinkMetrics>();
    metrics.reserve(links.size());
    for (const auto& [port, connection] : links) {
        const auto& link = connection.link;
        metrics.push_back({device_id,
                           connection.destination,
                           port,
                           link->get_link_class(),
                           link->get_transmitted_bytes(),
                           link->get_transmitted_messages(),
                           link->get_peak_outstanding_bytes(),
                           link->get_busy_time()});
    }
    return metrics;
}

LinkClass Device::get_link_class(const LinkId link_id) const noexcept {
    assert(links.find(link_id) != links.end());
    return links.at(link_id).link->get_link_class();
}

bool Device::link_connects(const LinkId link_id, const DeviceId destination) const noexcept {
    const auto connection = links.find(link_id);
    return connection != links.end() && connection->second.destination == destination;
}

uint64_t Device::get_outstanding_bytes(const LinkId link_id) const noexcept {
    assert(links.find(link_id) != links.end());
    return links.at(link_id).link->get_outstanding_bytes();
}

Bandwidth Device::get_link_bandwidth(const LinkId link_id) const noexcept {
    assert(links.find(link_id) != links.end());
    return links.at(link_id).link->get_bandwidth();
}

Latency Device::get_link_latency(const LinkId link_id) const noexcept {
    assert(links.find(link_id) != links.end());
    return links.at(link_id).link->get_latency();
}

void Device::send(std::unique_ptr<Chunk> chunk) noexcept {
    // assert the validity of the chunk
    assert(chunk != nullptr);

    // assert this node is the current source of the chunk
    assert(chunk->current_device()->get_id() == device_id);

    // assert the chunk hasn't arrived its final destination yet
    assert(!chunk->arrived_dest());

    // get next dest
    const auto next_dest_id = chunk->next_device()->get_id();
    auto link_id = chunk->current_link();
    if (link_id == AutomaticLink) {
        link_id = unique_link_to(next_dest_id);
    }

    // assert the next dest is connected to this node
    assert(link_connects(link_id, next_dest_id));

    // send the chunk to the next dest
    // delegate this task to the link
    links.at(link_id).link->send(std::move(chunk));
}

LinkId Device::connect(const DeviceId id, const Bandwidth bandwidth, const Latency latency,
                       const LinkClass link_class) noexcept {
    assert(id >= 0);
    assert(bandwidth > 0);
    assert(latency >= 0);

    // create link
    const auto link_id = next_link_id++;
    links.emplace(link_id, Connection{id, std::make_shared<Link>(bandwidth, latency, link_class)});
    return link_id;
}

void Device::reserve(const LinkId requested_link_id, const DeviceId destination,
                     const ChunkSize chunk_size) noexcept {
    const auto link_id = requested_link_id == AutomaticLink
                             ? unique_link_to(destination)
                             : requested_link_id;
    assert(link_connects(link_id, destination));
    links.at(link_id).link->reserve(chunk_size);
}

LinkId Device::unique_link_to(const DeviceId dest) const noexcept {
    auto match = AutomaticLink;
    for (const auto& [port, connection] : links) {
        if (connection.destination != dest) {
            continue;
        }
        if (match != AutomaticLink) {
            std::cerr << "[Error] (network/analytical/congestion_aware) route from device "
                      << device_id << " to " << dest
                      << " must select an exact parallel-link port" << std::endl;
            std::abort();
        }
        match = port;
    }
    if (match == AutomaticLink) {
        std::cerr << "[Error] (network/analytical/congestion_aware) no link from device "
                  << device_id << " to " << dest << std::endl;
        std::abort();
    }
    return match;
}
