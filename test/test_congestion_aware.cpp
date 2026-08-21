/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "common/EventQueue.h"
#include "common/NetworkParser.h"
#include "common/Type.h"
#include "congestion_aware/Chunk.h"
#include "congestion_aware/Helper.h"
#include "congestion_aware/Hybrid2D.h"
#include "congestion_aware/Mesh2D.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

class TestNetworkAnalyticalCongestionAware : public ::testing::Test {
  protected:
    void SetUp() override {
        // set event queue
        event_queue = std::make_shared<EventQueue>();
        Topology::set_event_queue(event_queue);

        // set chunk size
        chunk_size = 1'048'576;  // 1 MB
    }

    std::shared_ptr<EventQueue> event_queue;

    static void callback(void* const arg) {}

    ChunkSize chunk_size;
};

namespace {

std::vector<DeviceId> route_ids(const Route& route) {
    auto ids = std::vector<DeviceId>();
    ids.reserve(route.size());
    for (const auto& device : route) {
        ids.push_back(device->get_id());
    }
    return ids;
}

bool has_outgoing_link(const std::shared_ptr<Device>& source, const DeviceId destination) {
    const auto neighbors = source->get_connected_device_ids();
    return std::find(neighbors.begin(), neighbors.end(), destination) != neighbors.end();
}

std::vector<LinkClass> route_link_classes(const Route& route) {
    auto classes = std::vector<LinkClass>();
    if (route.empty()) {
        return classes;
    }
    auto current = route.begin();
    auto next = std::next(current);
    while (next != route.end()) {
        EXPECT_NE(current->outgoing_link, AutomaticLink);
        EXPECT_TRUE(current->device->link_connects(current->outgoing_link,
                                                   next->device->get_id()));
        classes.push_back(current->device->get_link_class(current->outgoing_link));
        ++current;
        ++next;
    }
    return classes;
}

int count_links(const std::vector<LinkMetrics>& metrics, const LinkClass link_class) {
    return static_cast<int>(std::count_if(
        metrics.begin(), metrics.end(), [link_class](const LinkMetrics& metric) {
            return metric.link_class == link_class;
        }));
}

}  // namespace

TEST_F(TestNetworkAnalyticalCongestionAware, HybridTopologiesHaveExactPhysicalPorts) {
    constexpr auto side = 4;
    constexpr auto npus = side * side;
    const auto expected_base_links = 4 * side * (side - 1);

    for (const auto fabric : {Hybrid2D::ExtraFabric::RowRing,
                              Hybrid2D::ExtraFabric::Switch}) {
        const auto topology = Hybrid2D(npus, 200.0, 1'000.0, fabric,
                                       Hybrid2D::RoutingPolicy::Static);
        const auto metrics = topology.get_link_metrics();
        EXPECT_EQ(count_links(metrics, LinkClass::BaseMesh), expected_base_links);

        if (fabric == Hybrid2D::ExtraFabric::RowRing) {
            EXPECT_EQ(topology.get_devices_count(), npus);
            EXPECT_EQ(count_links(metrics, LinkClass::RowRing), 2 * npus);
        } else {
            EXPECT_EQ(topology.get_devices_count(), npus + 2);
            EXPECT_EQ(count_links(metrics, LinkClass::SwitchUplink), 4 * npus);
        }

        for (auto npu = 0; npu < npus; ++npu) {
            const auto x = npu % side;
            const auto y = npu / side;
            const auto base_degree = 4 - (x == 0) - (x == side - 1) -
                                     (y == 0) - (y == side - 1);
            const auto degree = topology.route(npu, npu).front()
                                    ->get_connected_device_ids().size();
            EXPECT_EQ(degree, base_degree + 2) << "NPU " << npu;
        }
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware, RowRingParallelPortsRemainDistinct) {
    auto topology = Hybrid2D(16, 200.0, 1'000.0,
                             Hybrid2D::ExtraFabric::RowRing,
                             Hybrid2D::RoutingPolicy::Static);
    const auto source = topology.route(1, 1).front().device;
    auto classes_to_two = std::vector<LinkClass>();
    for (const auto& metric : source->get_link_metrics()) {
        if (metric.destination == 2) {
            classes_to_two.push_back(metric.link_class);
        }
    }
    std::sort(classes_to_two.begin(), classes_to_two.end());
    ASSERT_EQ(classes_to_two.size(), 2);
    EXPECT_NE(classes_to_two[0], classes_to_two[1]);
    EXPECT_NE(std::find(classes_to_two.begin(), classes_to_two.end(),
                        LinkClass::BaseMesh), classes_to_two.end());
    EXPECT_NE(std::find(classes_to_two.begin(), classes_to_two.end(),
                        LinkClass::RowRing), classes_to_two.end());

    EXPECT_DEATH(
        {
            auto ambiguous = Route();
            ambiguous.emplace_back(topology.route(1, 1).front().device);
            ambiguous.emplace_back(topology.route(2, 2).front().device);
            auto chunk = std::make_unique<Chunk>(chunk_size, ambiguous, callback, nullptr);
            topology.send(std::move(chunk));
        },
        "must select an exact parallel-link port");
}

TEST_F(TestNetworkAnalyticalCongestionAware, HybridStaticRoutesAreDeterministicAndValid) {
    const auto row_ring = Hybrid2D(16, 200.0, 1'000.0,
                                   Hybrid2D::ExtraFabric::RowRing,
                                   Hybrid2D::RoutingPolicy::Static);
    EXPECT_EQ(route_link_classes(row_ring.route(0, 3, chunk_size)),
              std::vector<LinkClass>({LinkClass::RowRing}));
    EXPECT_EQ(route_link_classes(row_ring.route(0, 1, chunk_size)),
              std::vector<LinkClass>({LinkClass::BaseMesh}));
    EXPECT_EQ(route_link_classes(row_ring.route(1, 2, chunk_size)),
              std::vector<LinkClass>({LinkClass::RowRing}));
    EXPECT_EQ(route_link_classes(row_ring.route(0, 12, chunk_size)),
              std::vector<LinkClass>(3, LinkClass::BaseMesh));

    const auto faster_extra = Hybrid2D(16, 200.0, 1'000.0,
                                       Hybrid2D::ExtraFabric::RowRing,
                                       Hybrid2D::RoutingPolicy::Static,
                                       400.0, 250.0);
    const auto faster_path = faster_extra.route(0, 3, chunk_size);
    ASSERT_EQ(faster_path.size(), 2);
    EXPECT_EQ(faster_path.front().device->get_link_bandwidth(
                  faster_path.front().outgoing_link),
              400.0);
    EXPECT_EQ(faster_path.front().device->get_link_latency(
                  faster_path.front().outgoing_link),
              250.0);

    const auto mesh_switch = Hybrid2D(16, 200.0, 1'000.0,
                                     Hybrid2D::ExtraFabric::Switch,
                                     Hybrid2D::RoutingPolicy::Static);
    EXPECT_EQ(route_link_classes(mesh_switch.route(0, 1, chunk_size)),
              std::vector<LinkClass>({LinkClass::BaseMesh}));
    EXPECT_EQ(route_link_classes(mesh_switch.route(0, 15, chunk_size)),
              std::vector<LinkClass>(2, LinkClass::SwitchUplink));
    EXPECT_EQ(route_ids(mesh_switch.route(0, 15, chunk_size)),
              route_ids(mesh_switch.route(0, 15, chunk_size)));

    for (const auto* topology : {&row_ring, &mesh_switch}) {
        for (auto source = 0; source < 16; ++source) {
            for (auto destination = 0; destination < 16; ++destination) {
                const auto path = topology->route(source, destination, chunk_size);
                EXPECT_EQ(path.front()->get_id(), source);
                EXPECT_EQ(path.back()->get_id(), destination);
                static_cast<void>(route_link_classes(path));
            }
        }
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware, AdaptiveRoutingAccountsForAssignedTraffic) {
    for (const auto fabric : {Hybrid2D::ExtraFabric::RowRing,
                              Hybrid2D::ExtraFabric::Switch}) {
        auto topology = Hybrid2D(16, 200.0, 1'000.0, fabric,
                                 Hybrid2D::RoutingPolicy::Adaptive);
        const auto destination = fabric == Hybrid2D::ExtraFabric::RowRing ? 1 : 15;
        const auto first = topology.route(0, destination, chunk_size);
        const auto first_ids = route_ids(first);
        const auto first_classes = route_link_classes(first);
        auto chunk = std::make_unique<Chunk>(chunk_size, first, callback, nullptr);
        topology.send(std::move(chunk));

        const auto second = topology.route(0, destination, chunk_size);
        const auto second_ids = route_ids(second);
        const auto second_classes = route_link_classes(second);
        if (fabric == Hybrid2D::ExtraFabric::RowRing) {
            EXPECT_EQ(first_classes, std::vector<LinkClass>({LinkClass::BaseMesh}));
            EXPECT_EQ(second_classes, std::vector<LinkClass>({LinkClass::RowRing}));
        } else {
            EXPECT_EQ(first_classes, std::vector<LinkClass>(2, LinkClass::SwitchUplink));
            EXPECT_EQ(second_classes, std::vector<LinkClass>(2, LinkClass::SwitchUplink));
            EXPECT_NE(first_ids[1], second_ids[1]);
        }

        while (!event_queue->finished()) {
            event_queue->proceed();
        }
        for (const auto& metric : topology.get_link_metrics()) {
            EXPECT_EQ(metric.peak_outstanding_bytes == 0, metric.bytes == 0);
        }
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware, MeshAndTorusGraphStructure) {
    constexpr auto side = 4;
    constexpr auto npus = side * side;

    for (const auto wraparound : {false, true}) {
        const auto topology = Mesh2D(npus, 200.0, 1'000.0, wraparound);
        auto directed_links = 0;

        for (auto id = 0; id < npus; ++id) {
            const auto self_route = topology.route(id, id);
            ASSERT_EQ(self_route.size(), 1);
            const auto degree = self_route.front()->get_connected_device_ids().size();
            directed_links += static_cast<int>(degree);

            const auto x = id % side;
            const auto y = id / side;
            const auto expected_degree = wraparound
                                             ? 4
                                             : 4 - (x == 0) - (x == side - 1) -
                                                   (y == 0) - (y == side - 1);
            EXPECT_EQ(degree, expected_degree) << "device " << id;
        }

        const auto expected_links = wraparound ? 4 * side * side : 4 * side * (side - 1);
        EXPECT_EQ(directed_links, expected_links);
        EXPECT_EQ(topology.get_basic_topology_type(),
                  wraparound ? TopologyBuildingBlock::Torus2D
                             : TopologyBuildingBlock::Mesh2D);
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware, MeshAndTorusRoutesAreValidAndMinimal) {
    for (const auto side : {4, 8, 16}) {
        const auto npus = side * side;
        for (const auto wraparound : {false, true}) {
            const auto topology = Mesh2D(npus, 200.0, 1'000.0, wraparound);

            for (auto source = 0; source < npus; ++source) {
                for (auto destination = 0; destination < npus; ++destination) {
                    const auto route = topology.route(source, destination);
                    const auto ids = route_ids(route);
                    ASSERT_FALSE(ids.empty());
                    EXPECT_EQ(ids.front(), source);
                    EXPECT_EQ(ids.back(), destination);
                    EXPECT_EQ(ids, route_ids(topology.route(source, destination)));

                    const auto sx = source % side;
                    const auto sy = source / side;
                    const auto dx = destination % side;
                    const auto dy = destination / side;
                    auto x_distance = std::abs(dx - sx);
                    auto y_distance = std::abs(dy - sy);
                    if (wraparound) {
                        x_distance = std::min(x_distance, side - x_distance);
                        y_distance = std::min(y_distance, side - y_distance);
                    }
                    EXPECT_EQ(ids.size() - 1,
                              static_cast<std::size_t>(x_distance + y_distance));

                    auto current = route.begin();
                    auto next = current;
                    if (next != route.end()) {
                        ++next;
                    }
                    while (next != route.end()) {
                        EXPECT_TRUE(has_outgoing_link(*current, (*next)->get_id()))
                            << "missing hop " << (*current)->get_id() << " -> "
                            << (*next)->get_id();
                        ++current;
                        ++next;
                    }
                }
            }
        }
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware, TorusAntipodalTiesAreBalanced) {
    constexpr auto side = 16;
    constexpr auto npus = side * side;
    const auto topology = Mesh2D(npus, 200.0, 1'000.0, true);
    auto x_forward = 0;
    auto x_backward = 0;
    auto y_forward = 0;
    auto y_backward = 0;

    for (auto source = 0; source < npus; ++source) {
        const auto x = source % side;
        const auto y = source / side;

        const auto x_destination = y * side + ((x + side / 2) % side);
        const auto x_route = route_ids(topology.route(source, x_destination));
        const auto x_next = x_route[1] % side;
        x_forward += x_next == (x + 1) % side;
        x_backward += x_next == (x - 1 + side) % side;

        const auto y_destination = ((y + side / 2) % side) * side + x;
        const auto y_route = route_ids(topology.route(source, y_destination));
        const auto y_next = y_route[1] / side;
        y_forward += y_next == (y + 1) % side;
        y_backward += y_next == (y - 1 + side) % side;
    }

    EXPECT_EQ(x_forward, x_backward);
    EXPECT_EQ(y_forward, y_backward);
}

TEST_F(TestNetworkAnalyticalCongestionAware, SnakeEmbeddingIsAHamiltonianCycle) {
    for (const auto side : {4, 8, 16}) {
        const auto npus = side * side;
        for (const auto wraparound : {false, true}) {
            const auto topology = Mesh2D(npus, 200.0, 1'000.0, wraparound,
                                         Mesh2D::Embedding::Snake);
            for (auto source = 0; source < npus; ++source) {
                const auto destination = (source + 1) % npus;
                const auto route = topology.route(source, destination);
                ASSERT_EQ(route.size(), 2) << "logical edge " << source << " -> "
                                           << destination;
                EXPECT_TRUE(has_outgoing_link(route.front(), route.back()->get_id()));
            }
        }
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware, OddSnakeExtentIsRejected) {
    EXPECT_DEATH(
        { const auto topology = Mesh2D(8, 200.0, 1'000.0, false); },
        "perfect-square npus_count");
    EXPECT_DEATH(
        { const auto topology = Mesh2D(9, 200.0, 1'000.0, false,
                                       Mesh2D::Embedding::Snake); },
        "Snake placement requires an even grid extent");
}

TEST_F(TestNetworkAnalyticalCongestionAware, Ring) {
    /// setup
    const auto network_parser = NetworkParser("../../input/Ring.yml");
    const auto topology = construct_topology(network_parser);

    /// message settings
    auto route = topology->route(1, 4);
    auto chunk = std::make_unique<Chunk>(chunk_size, route, callback, nullptr);

    // send a chunk
    topology->send(std::move(chunk));

    /// Run simulation
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    /// test
    const auto simulation_time = event_queue->get_current_time();
    EXPECT_EQ(simulation_time, 60'093);
}

TEST_F(TestNetworkAnalyticalCongestionAware, FullyConnected) {
    /// setup
    const auto network_parser = NetworkParser("../../input/FullyConnected.yml");
    const auto topology = construct_topology(network_parser);

    /// message settings
    auto route = topology->route(1, 4);
    auto chunk = std::make_unique<Chunk>(chunk_size, route, callback, nullptr);

    // send a chunk
    topology->send(std::move(chunk));

    /// Run simulation
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    /// test
    const auto simulation_time = event_queue->get_current_time();
    EXPECT_EQ(simulation_time, 20'031);
}

TEST_F(TestNetworkAnalyticalCongestionAware, Switch) {
    /// setup
    const auto network_parser = NetworkParser("../../input/Switch.yml");
    const auto topology = construct_topology(network_parser);

    /// message settings
    auto route = topology->route(1, 4);
    auto chunk = std::make_unique<Chunk>(chunk_size, route, callback, nullptr);

    // send a chunk
    topology->send(std::move(chunk));

    /// Run simulation
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    /// test
    const auto simulation_time = event_queue->get_current_time();
    EXPECT_EQ(simulation_time, 40'062);
}

TEST_F(TestNetworkAnalyticalCongestionAware, AllGatherOnRing) {
    /// setup
    const auto network_parser = NetworkParser("../../input/Ring.yml");
    const auto topology = construct_topology(network_parser);
    const auto npus_count = topology->get_npus_count();

    /// message settings
    const auto chunk_size = 1'048'576;  // 1 MB

    /// Run All-Gather
    for (int i = 0; i < npus_count; i++) {
        for (int j = 0; j < npus_count; j++) {
            if (i == j) {
                continue;
            }

            // crate a chunk
            auto route = topology->route(i, j);
            auto* event_queue_ptr = static_cast<void*>(event_queue.get());
            auto chunk = std::make_unique<Chunk>(chunk_size, route, callback, nullptr);

            // send a chunk
            topology->send(std::move(chunk));
        }
    }

    /// Run simulation
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    /// test
    const auto simulation_time = event_queue->get_current_time();
    EXPECT_EQ(simulation_time, 704'116);
}
