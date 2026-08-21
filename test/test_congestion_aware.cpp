/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "common/EventQueue.h"
#include "common/NetworkParser.h"
#include "common/Type.h"
#include "congestion_aware/Chunk.h"
#include "congestion_aware/Helper.h"
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

}  // namespace

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
