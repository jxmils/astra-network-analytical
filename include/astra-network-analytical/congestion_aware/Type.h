/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include "common/Type.h"
#include <cstdint>
#include <list>
#include <memory>
#include <string_view>

namespace NetworkAnalyticalCongestionAware {

/// Forward declarations of network components
class Chunk;
class Link;
class Device;

/// Link IDs are local to a device and identify an exact outgoing physical port.
using LinkId = int;
constexpr LinkId AutomaticLink = -1;

enum class LinkClass { Generic, BaseMesh, RowRing, SwitchUplink };

[[nodiscard]] constexpr std::string_view link_class_name(const LinkClass link_class) noexcept {
    switch (link_class) {
    case LinkClass::Generic:
        return "generic";
    case LinkClass::BaseMesh:
        return "base_mesh";
    case LinkClass::RowRing:
        return "row_ring";
    case LinkClass::SwitchUplink:
        return "switch_uplink";
    }
    return "unknown";
}

struct LinkMetrics {
    NetworkAnalytical::DeviceId source;
    NetworkAnalytical::DeviceId destination;
    LinkId port;
    LinkClass link_class;
    uint64_t bytes;
    uint64_t messages;
    uint64_t peak_outstanding_bytes;
    NetworkAnalytical::EventTime busy_time;
};

/**
 * A route hop names both a device and the exact outgoing port to use there.
 * The final hop has no outgoing port. AutomaticLink preserves the old route
 * construction style for topologies with one link per destination.
 */
struct RouteHop {
    RouteHop(std::shared_ptr<Device> device, LinkId outgoing_link = AutomaticLink) noexcept
        : device(std::move(device)), outgoing_link(outgoing_link) {}

    [[nodiscard]] Device* operator->() const noexcept {
        return device.get();
    }

    [[nodiscard]] operator const std::shared_ptr<Device>&() const noexcept {
        return device;
    }

    std::shared_ptr<Device> device;
    LinkId outgoing_link;
};

using Route = std::list<RouteHop>;

}  // namespace NetworkAnalyticalCongestionAware
