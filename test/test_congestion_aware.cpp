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
#include "congestion_aware/HierarchicalCluster.h"
#include "congestion_aware/Mesh2D.h"
#include "congestion_aware/Mesh3D.h"
#include "congestion_aware/MultiPlaneSwitch.h"
#include "congestion_aware/OcsSwitch.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <set>
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

struct ArrivalObservation {
    std::shared_ptr<EventQueue> event_queue;
    EventTime arrival_time = -1;
};

void record_arrival(void* const arg) {
    auto* const observation = static_cast<ArrivalObservation*>(arg);
    observation->arrival_time = observation->event_queue->get_current_time();
}

std::string write_ocs_test_plan(const std::string& name,
                                const bool initial_reconfiguration = false) {
    const auto path = "/tmp/" + name + ".json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-ocs-plan",
  "version": 1,
  "endpoints": 16,
  "planes": 6,
  "link_bandwidth_GBps": 1.0,
  "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0,
  "initial_reconfiguration": )" << (initial_reconfiguration ? "true" : "false") << R"(,
  "rounds": [
    {"index": 0, "configurations": [
      {"plane": 0, "circuits": [{"source": 0, "destination": 1, "bytes": 100}]},
      {"plane": 1, "circuits": [{"source": 2, "destination": 3, "bytes": 100}]}
    ]},
    {"index": 1, "configurations": [
      {"plane": 0, "circuits": [{"source": 0, "destination": 2, "bytes": 100}]}
    ]}
  ]
})";
    output.close();
    return path;
}

std::string write_hybrid_ocs_test_plan() {
    const auto path = "/tmp/analytical-torus-ocs.json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-ocs-plan", "version": 1, "endpoints": 16, "planes": 2,
  "link_bandwidth_GBps": 1.0, "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0, "initial_reconfiguration": false,
  "rounds": [{"index": 0, "configurations": [
    {"plane": 0, "circuits": [{"source": 0, "destination": 10, "bytes": 100}]}
  ]}]
})";
    output.close();
    return path;
}

}  // namespace

TEST_F(TestNetworkAnalyticalCongestionAware, OcsSwitchEnforcesRoundsAndReconfiguration) {
    const auto plan = write_ocs_test_plan("analytical-ocs-rounds");
    auto topology = OcsSwitch(16, 1.0, 5.0, plan);
    auto first = ArrivalObservation{event_queue};
    auto parallel = ArrivalObservation{event_queue};
    auto second_round = ArrivalObservation{event_queue};

    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 1, 100), record_arrival, &first));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(2, 3, 100), record_arrival, &parallel));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 2, 100), record_arrival, &second_round));
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    EXPECT_EQ(first.arrival_time, parallel.arrival_time);
    EXPECT_EQ(first.arrival_time, 104);
    EXPECT_EQ(second_round.arrival_time, 205);
    EXPECT_EQ(topology.get_completed_rounds(), 2);
    EXPECT_EQ(topology.get_reconfiguration_count(), 1);
    EXPECT_EQ(topology.get_reconfiguration_time(), 7);
    EXPECT_EQ(topology.get_scheduled_bytes(), 300);
    EXPECT_EQ(topology.get_transmitted_bytes(), 300);
    std::remove(plan.c_str());
}

TEST_F(TestNetworkAnalyticalCongestionAware, OcsSwitchChargesOptionalInitialConfiguration) {
    const auto plan = write_ocs_test_plan("analytical-ocs-initial", true);
    auto topology = OcsSwitch(16, 1.0, 5.0, plan);
    auto observation = ArrivalObservation{event_queue};
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 1, 100), record_arrival, &observation));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(2, 3, 100), callback, nullptr));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 2, 100), callback, nullptr));
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    EXPECT_EQ(observation.arrival_time, 111);
    EXPECT_EQ(topology.get_reconfiguration_count(), 2);
    EXPECT_EQ(topology.get_reconfiguration_time(), 14);
    std::remove(plan.c_str());
}

TEST_F(TestNetworkAnalyticalCongestionAware, TorusOcsUsesDirectAndCircuitPorts) {
    const auto plan = write_hybrid_ocs_test_plan();
    auto topology = OcsSwitch(16, 1.0, 5.0, plan, 2, true);
    auto direct = ArrivalObservation{event_queue};
    auto circuit = ArrivalObservation{event_queue};
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 1, 100), record_arrival, &direct));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 10, 100), record_arrival, &circuit));
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    EXPECT_EQ(direct.arrival_time, 98);
    EXPECT_EQ(circuit.arrival_time, 104);
    EXPECT_EQ(topology.get_scheduled_bytes(), 100);
    EXPECT_EQ(topology.get_transmitted_bytes(), 100);
    EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::BaseMesh), 64);
    EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::SwitchUplink), 64);
    std::remove(plan.c_str());
}

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

TEST_F(TestNetworkAnalyticalCongestionAware, DirectPreferenceAvoidsMarginalSwitching) {
    const auto route_after_reservation = [this](const ChunkSize reserved_bytes) {
        event_queue = std::make_shared<EventQueue>();
        Topology::set_event_queue(event_queue);
        auto topology = Hybrid2D(
            16, 200.0, 1'000.0, Hybrid2D::ExtraFabric::Switch,
            Hybrid2D::RoutingPolicy::DirectPreferredAdaptive,
            200.0, 1'000.0, 1.10);
        const auto reserved_route = topology.route(0, 2, reserved_bytes);
        EXPECT_EQ(route_link_classes(reserved_route),
                  std::vector<LinkClass>(2, LinkClass::BaseMesh));
        topology.send(std::make_unique<Chunk>(
            reserved_bytes, reserved_route, callback, nullptr));
        return route_link_classes(topology.route(0, 2, chunk_size));
    };

    EXPECT_EQ(route_after_reservation(4'096),
              std::vector<LinkClass>(2, LinkClass::BaseMesh));
    EXPECT_EQ(route_after_reservation(chunk_size),
              std::vector<LinkClass>(2, LinkClass::SwitchUplink));

    event_queue = std::make_shared<EventQueue>();
    Topology::set_event_queue(event_queue);
    auto ordinary_adaptive = Hybrid2D(
        16, 200.0, 1'000.0, Hybrid2D::ExtraFabric::Switch,
        Hybrid2D::RoutingPolicy::Adaptive);
    const auto reserved_route = ordinary_adaptive.route(0, 2, 4'096);
    ordinary_adaptive.send(std::make_unique<Chunk>(
        4'096, reserved_route, callback, nullptr));
    EXPECT_EQ(route_link_classes(ordinary_adaptive.route(0, 2, chunk_size)),
              std::vector<LinkClass>(2, LinkClass::SwitchUplink));
}

TEST_F(TestNetworkAnalyticalCongestionAware, OfflinePlanSelectsExactRoutes) {
    const auto plan_path = std::string("hybrid-offline-plan-test.txt");
    {
        auto output = std::ofstream(plan_path);
        ASSERT_TRUE(output.good());
        output << "0 15 " << chunk_size << " DIRECT\n"
               << "0 15 " << chunk_size << " SWITCH0\n"
               << "0 15 " << chunk_size << " SWITCH1\n";
    }
    auto topology = Hybrid2D(
        16, 200.0, 1'000.0, Hybrid2D::ExtraFabric::Switch,
        Hybrid2D::RoutingPolicy::OfflineOracle,
        200.0, 1'000.0, 1.10, plan_path);

    EXPECT_EQ(route_link_classes(topology.route(0, 15, chunk_size)),
              std::vector<LinkClass>(6, LinkClass::BaseMesh));
    EXPECT_EQ(route_ids(topology.route(0, 15, chunk_size))[1], 16);
    EXPECT_EQ(route_ids(topology.route(0, 15, chunk_size))[1], 17);
    EXPECT_EQ(std::remove(plan_path.c_str()), 0);
}

TEST_F(TestNetworkAnalyticalCongestionAware, ExtraLinkLatencyChangesPhysicalArrivalDelay) {
    const auto measure = [this](const Hybrid2D::ExtraFabric fabric,
                                const Latency extra_latency,
                                const DeviceId destination) {
        event_queue = std::make_shared<EventQueue>();
        Topology::set_event_queue(event_queue);
        auto topology = Hybrid2D(16, 200.0, 1'000.0, fabric,
                                 Hybrid2D::RoutingPolicy::Static,
                                 200.0, extra_latency);
        const auto path = topology.route(0, destination, chunk_size);
        ArrivalObservation observation{event_queue};
        auto chunk = std::make_unique<Chunk>(
            chunk_size, path, record_arrival, &observation);
        topology.send(std::move(chunk));
        while (!event_queue->finished()) {
            event_queue->proceed();
        }
        EXPECT_GE(observation.arrival_time, 0);
        return observation.arrival_time;
    };

    const auto row_zero = measure(Hybrid2D::ExtraFabric::RowRing, 0.0, 3);
    const auto row_one = measure(Hybrid2D::ExtraFabric::RowRing, 1'000.0, 3);
    const auto row_two = measure(Hybrid2D::ExtraFabric::RowRing, 2'000.0, 3);
    EXPECT_EQ(row_one - row_zero, 1'000);
    EXPECT_EQ(row_two - row_one, 1'000);

    const auto switch_zero = measure(Hybrid2D::ExtraFabric::Switch, 0.0, 15);
    const auto switch_one = measure(Hybrid2D::ExtraFabric::Switch, 1'000.0, 15);
    const auto switch_two = measure(Hybrid2D::ExtraFabric::Switch, 2'000.0, 15);
    EXPECT_EQ(switch_one - switch_zero, 2'000);
    EXPECT_EQ(switch_two - switch_one, 2'000);
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

TEST_F(TestNetworkAnalyticalCongestionAware, ThreeDimensionalFabricsHaveExactPorts) {
    constexpr auto extent = 4;
    constexpr auto npus = extent * extent * extent;
    for (const auto wraparound : {false, true}) {
        const auto topology = Mesh3D(npus, 200.0, 1'000.0, wraparound);
        auto directed_links = 0;
        for (auto id = 0; id < npus; ++id) {
            const auto neighbors = topology.route(id, id).front()
                                       ->get_connected_device_ids();
            const auto distinct = std::set<DeviceId>(neighbors.begin(), neighbors.end());
            EXPECT_EQ(distinct.size(), neighbors.size());
            const auto x = id % extent;
            const auto y = (id / extent) % extent;
            const auto z = id / (extent * extent);
            const auto expected_degree = wraparound
                                             ? 6
                                             : 6 - (x == 0) - (x == extent - 1) -
                                                   (y == 0) - (y == extent - 1) -
                                                   (z == 0) - (z == extent - 1);
            EXPECT_EQ(neighbors.size(), expected_degree) << "device " << id;
            directed_links += static_cast<int>(neighbors.size());
        }
        EXPECT_EQ(directed_links,
                  wraparound ? 6 * npus
                             : 6 * extent * extent * (extent - 1));
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware, ThreeDimensionalRoutesAreMinimal) {
    constexpr auto extent = 4;
    constexpr auto npus = extent * extent * extent;
    for (const auto wraparound : {false, true}) {
        const auto topology = Mesh3D(npus, 200.0, 1'000.0, wraparound);
        for (auto source = 0; source < npus; ++source) {
            for (auto destination = 0; destination < npus; ++destination) {
                const auto path = topology.route(source, destination);
                const auto ids = route_ids(path);
                EXPECT_EQ(ids.front(), source);
                EXPECT_EQ(ids.back(), destination);
                EXPECT_EQ(ids, route_ids(topology.route(source, destination)));
                auto distance = 0;
                for (const auto stride : {1, extent, extent * extent}) {
                    const auto source_coordinate = (source / stride) % extent;
                    const auto destination_coordinate =
                        (destination / stride) % extent;
                    auto axis_distance = std::abs(destination_coordinate -
                                                  source_coordinate);
                    if (wraparound) {
                        axis_distance = std::min(axis_distance,
                                                 extent - axis_distance);
                    }
                    distance += axis_distance;
                }
                EXPECT_EQ(ids.size() - 1, static_cast<std::size_t>(distance));
                auto current = path.begin();
                auto next = std::next(current);
                while (next != path.end()) {
                    EXPECT_TRUE(has_outgoing_link(current->device,
                                                  next->device->get_id()));
                    ++current;
                    ++next;
                }
            }
        }
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       GroupedThreeDimensionalFabricsChangeOnlyLogicalDimensions) {
    constexpr auto npus = 64;
    for (const auto wraparound : {false, true}) {
        const auto flat = Mesh3D(npus, 200.0, 1'000.0, wraparound);
        const auto grouped = Mesh3D(npus, 200.0, 1'000.0, wraparound, true);
        EXPECT_EQ(flat.get_npus_count_per_dim(), std::vector<int>({npus}));
        EXPECT_EQ(grouped.get_npus_count_per_dim(),
                  std::vector<int>({4, 4, 4}));
        EXPECT_EQ(grouped.get_bandwidth_per_dim(),
                  std::vector<Bandwidth>({200.0, 200.0, 200.0}));
        const auto flat_links = flat.get_link_metrics();
        const auto grouped_links = grouped.get_link_metrics();
        ASSERT_EQ(flat_links.size(), grouped_links.size());
        for (std::size_t index = 0; index < flat_links.size(); ++index) {
            EXPECT_EQ(flat_links[index].source, grouped_links[index].source);
            EXPECT_EQ(flat_links[index].destination,
                      grouped_links[index].destination);
            EXPECT_EQ(flat_links[index].port, grouped_links[index].port);
            EXPECT_EQ(flat_links[index].link_class,
                      grouped_links[index].link_class);
        }
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware, SixPortBaselinesUseExactEndpointPorts) {
    constexpr auto npus = 64;
    const auto hybrid = Hybrid2D(
        npus, 200.0, 1'000.0, Hybrid2D::ExtraFabric::Switch,
        Hybrid2D::RoutingPolicy::Adaptive, 200.0, 1'000.0, 1.10, "", true);
    EXPECT_EQ(count_links(hybrid.get_link_metrics(), LinkClass::BaseMesh),
              4 * npus);
    EXPECT_EQ(count_links(hybrid.get_link_metrics(), LinkClass::SwitchUplink),
              4 * npus);
    for (auto npu = 0; npu < npus; ++npu) {
        EXPECT_EQ(hybrid.route(npu, npu).front()
                      ->get_connected_device_ids().size(),
                  6);
    }

    auto full_switch = MultiPlaneSwitch(npus, 200.0, 1'000.0);
    EXPECT_EQ(count_links(full_switch.get_link_metrics(), LinkClass::SwitchUplink),
              2 * npus * MultiPlaneSwitch::Planes);
    for (auto npu = 0; npu < npus; ++npu) {
        EXPECT_EQ(full_switch.route(npu, npu).front()
                      ->get_connected_device_ids().size(),
                  MultiPlaneSwitch::Planes);
    }

    auto selected_planes = std::set<DeviceId>();
    for (auto request = 0; request < MultiPlaneSwitch::Planes; ++request) {
        const auto path = full_switch.route(0, 63, chunk_size);
        const auto ids = route_ids(path);
        ASSERT_EQ(ids.size(), 3);
        selected_planes.insert(ids[1]);
        full_switch.send(std::make_unique<Chunk>(
            chunk_size, path, callback, nullptr));
    }
    EXPECT_EQ(selected_planes.size(), MultiPlaneSwitch::Planes);
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       HierarchicalClusterHasExactResourcesAndLogicalDimensions) {
    constexpr auto npus = 64;
    const auto topology = HierarchicalCluster(
        npus, 900.0, 1'000.0, 100.0, 1'000.0, 4);
    const auto metrics = topology.get_link_metrics();

    EXPECT_EQ(topology.get_npus_count_per_dim(), std::vector<int>({8, 8}));
    EXPECT_EQ(topology.get_bandwidth_per_dim(),
              std::vector<Bandwidth>({900.0, 50.0}));
    EXPECT_EQ(topology.get_nodes_count(), 8);
    EXPECT_EQ(topology.get_nic_count(), 4);
    EXPECT_DOUBLE_EQ(topology.get_bandwidth_per_nic(), 100.0);
    EXPECT_DOUBLE_EQ(topology.get_scale_out_bandwidth_per_node(), 400.0);
    EXPECT_EQ(count_links(metrics, LinkClass::ScaleUp), 2 * npus);
    EXPECT_EQ(count_links(metrics, LinkClass::Gateway), 2 * 8 * 4);
    EXPECT_EQ(count_links(metrics, LinkClass::ScaleOut), 2 * 8 * 4);
    EXPECT_EQ(metrics.size(), 256);

    for (auto npu = 0; npu < npus; ++npu) {
        EXPECT_EQ(topology.route(npu, npu).front()
                      ->get_connected_device_ids().size(),
                  1);
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       HierarchicalClusterUsesEveryProvisionedNic) {
    constexpr auto nic_count = 4;
    const auto topology = HierarchicalCluster(
        64, 900.0, 1'000.0, 100.0, 1'000.0, nic_count);
    auto source_nics = std::set<DeviceId>();
    auto destination_nics = std::set<DeviceId>();
    for (auto local_rank = 0; local_rank < 8; ++local_rank) {
        const auto ids = route_ids(topology.route(local_rank, 8 + local_rank));
        ASSERT_EQ(ids.size(), 7);
        source_nics.insert(ids[2]);
        destination_nics.insert(ids[4]);
    }
    EXPECT_EQ(source_nics.size(), nic_count);
    EXPECT_EQ(destination_nics.size(), nic_count);
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       HierarchicalClusterStripesSyntheticBoundaryTraffic) {
    constexpr auto nic_count = 4;
    auto topology = HierarchicalCluster(
        64, 900.0, 1'000.0, 100.0, 1'000.0, nic_count);
    for (auto local_rank = 0; local_rank < 8; ++local_rank) {
        topology.send(std::make_unique<Chunk>(
            chunk_size, topology.route(local_rank, 8 + local_rank),
            callback, nullptr));
    }
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    const auto metrics = topology.get_link_metrics();
    auto gateway_bytes = ChunkSize{0};
    auto scale_out_bytes = ChunkSize{0};
    auto max_gateway_bytes = ChunkSize{0};
    auto max_scale_out_bytes = ChunkSize{0};
    for (const auto& metric : metrics) {
        if (metric.link_class == LinkClass::Gateway) {
            gateway_bytes += metric.bytes;
            max_gateway_bytes = std::max(max_gateway_bytes, metric.bytes);
        } else if (metric.link_class == LinkClass::ScaleOut) {
            scale_out_bytes += metric.bytes;
            max_scale_out_bytes = std::max(max_scale_out_bytes, metric.bytes);
        }
    }
    EXPECT_EQ(gateway_bytes, 16 * chunk_size);
    EXPECT_EQ(scale_out_bytes, 16 * chunk_size);
    EXPECT_EQ(max_gateway_bytes, 2 * chunk_size);
    EXPECT_EQ(max_scale_out_bytes, 2 * chunk_size);
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       HierarchicalClusterSeparatesLocalAndBoundaryRoutes) {
    const auto topology = HierarchicalCluster(
        64, 200.0, 1'000.0, 25.0, 1'000.0);
    const auto local = topology.route(0, 7);
    const auto boundary = topology.route(0, 8);

    EXPECT_EQ(route_ids(local), std::vector<DeviceId>({0, 64, 7}));
    EXPECT_EQ(route_link_classes(local),
              std::vector<LinkClass>(2, LinkClass::ScaleUp));
    EXPECT_EQ(route_ids(boundary),
              std::vector<DeviceId>({0, 64, 72, 80, 73, 65, 8}));
    EXPECT_EQ(route_link_classes(boundary),
              std::vector<LinkClass>({
                  LinkClass::ScaleUp, LinkClass::Gateway,
                  LinkClass::ScaleOut, LinkClass::ScaleOut,
                  LinkClass::Gateway, LinkClass::ScaleUp}));
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       HierarchicalClusterTelemetryDistinguishesBoundaryTraffic) {
    auto topology = HierarchicalCluster(
        64, 200.0, 1'000.0, 25.0, 1'000.0);
    const auto local = topology.route(0, 7);
    const auto boundary = topology.route(0, 8);
    topology.send(std::make_unique<Chunk>(chunk_size, local, callback, nullptr));
    topology.send(std::make_unique<Chunk>(chunk_size, boundary, callback, nullptr));

    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    const auto routes = topology.get_route_metrics();
    ASSERT_EQ(routes.size(), 2);
    const auto local_metric = std::find_if(
        routes.begin(), routes.end(), [](const RouteMetrics& metric) {
            return metric.route_class == RouteClass::Local;
        });
    const auto boundary_metric = std::find_if(
        routes.begin(), routes.end(), [](const RouteMetrics& metric) {
            return metric.route_class == RouteClass::Boundary;
        });
    ASSERT_NE(local_metric, routes.end());
    ASSERT_NE(boundary_metric, routes.end());
    EXPECT_EQ(local_metric->hops, 2);
    EXPECT_EQ(local_metric->payload_bytes, chunk_size);
    EXPECT_EQ(boundary_metric->hops, 6);
    EXPECT_EQ(boundary_metric->payload_bytes, chunk_size);
}

TEST_F(TestNetworkAnalyticalCongestionAware, TorusHybridAblationsKeepIdenticalHardware) {
    constexpr auto npus = 64;
    for (const auto policy : {Hybrid2D::RoutingPolicy::DirectOnly,
                              Hybrid2D::RoutingPolicy::SwitchOnly,
                              Hybrid2D::RoutingPolicy::Adaptive}) {
        auto topology = Hybrid2D(
            npus, 200.0, 1'000.0, Hybrid2D::ExtraFabric::Switch,
            policy, 200.0, 1'000.0, 1.10, "", true);
        EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::BaseMesh),
                  4 * npus);
        EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::SwitchUplink),
                  4 * npus);

        const auto path = topology.route(0, 1, chunk_size);
        const auto classes = route_link_classes(path);
        if (policy == Hybrid2D::RoutingPolicy::SwitchOnly) {
            EXPECT_EQ(classes, std::vector<LinkClass>(2, LinkClass::SwitchUplink));
        } else {
            EXPECT_EQ(classes, std::vector<LinkClass>({LinkClass::BaseMesh}));
        }
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       GroupedTorusHybridAblationsKeepIdenticalHardware) {
    constexpr auto npus = 64;
    for (const auto policy : {Hybrid2D::RoutingPolicy::DirectOnly,
                              Hybrid2D::RoutingPolicy::SwitchOnly,
                              Hybrid2D::RoutingPolicy::Adaptive}) {
        const auto topology = Hybrid2D(
            npus, 200.0, 1'000.0, Hybrid2D::ExtraFabric::Switch,
            policy, 200.0, 1'000.0, 1.10, "", true,
            Hybrid2D::LogicalShape::Grid);
        EXPECT_EQ(topology.get_npus_count_per_dim(),
                  std::vector<int>({8, 8}));
        EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::BaseMesh),
                  4 * npus);
        EXPECT_EQ(count_links(topology.get_link_metrics(),
                              LinkClass::SwitchUplink), 4 * npus);
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       GroupedTorusHybridChangesOnlyLogicalDimensions) {
    constexpr auto npus = 64;
    const auto flat = Hybrid2D(
        npus, 200.0, 1'000.0, Hybrid2D::ExtraFabric::Switch,
        Hybrid2D::RoutingPolicy::Adaptive, 200.0, 1'000.0, 1.10, "", true);
    const auto grouped = Hybrid2D(
        npus, 200.0, 1'000.0, Hybrid2D::ExtraFabric::Switch,
        Hybrid2D::RoutingPolicy::Adaptive, 200.0, 1'000.0, 1.10, "", true,
        Hybrid2D::LogicalShape::Grid);

    EXPECT_EQ(flat.get_npus_count_per_dim(), std::vector<int>({npus}));
    EXPECT_EQ(grouped.get_npus_count_per_dim(), std::vector<int>({8, 8}));
    EXPECT_EQ(grouped.get_bandwidth_per_dim(),
              std::vector<Bandwidth>({200.0, 200.0}));

    const auto flat_links = flat.get_link_metrics();
    const auto grouped_links = grouped.get_link_metrics();
    ASSERT_EQ(flat_links.size(), grouped_links.size());
    for (std::size_t index = 0; index < flat_links.size(); ++index) {
        EXPECT_EQ(flat_links[index].source, grouped_links[index].source);
        EXPECT_EQ(flat_links[index].destination, grouped_links[index].destination);
        EXPECT_EQ(flat_links[index].port, grouped_links[index].port);
        EXPECT_EQ(flat_links[index].link_class, grouped_links[index].link_class);
    }
}

TEST_F(TestNetworkAnalyticalCongestionAware, RouteAndQueueMetricsAreExact) {
    auto topology = Hybrid2D(
        64, 200.0, 1'000.0, Hybrid2D::ExtraFabric::Switch,
        Hybrid2D::RoutingPolicy::DirectOnly,
        200.0, 1'000.0, 1.10, "", true);
    const auto path = topology.route(0, 1, chunk_size);
    topology.send(std::make_unique<Chunk>(
        chunk_size, path, callback, nullptr));
    topology.send(std::make_unique<Chunk>(
        chunk_size, path, callback, nullptr));

    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    const auto routes = topology.get_route_metrics();
    ASSERT_EQ(routes.size(), 1);
    EXPECT_EQ(routes.front().route_class, RouteClass::Direct);
    EXPECT_EQ(routes.front().hops, 1);
    EXPECT_EQ(routes.front().messages, 2);
    EXPECT_EQ(routes.front().payload_bytes, 2 * chunk_size);
    EXPECT_EQ(routes.front().byte_hops, 2 * chunk_size);
    EXPECT_EQ(routes.front().propagation_time, 2'000);

    const auto serialization_time = routes.front().serialization_time / 2;
    const auto links = topology.get_link_metrics();
    const auto used = std::find_if(
        links.begin(), links.end(), [](const LinkMetrics& metric) {
            return metric.source == 0 && metric.destination == 1 && metric.bytes > 0;
        });
    ASSERT_NE(used, links.end());
    EXPECT_EQ(used->messages, 2);
    EXPECT_EQ(used->busy_time, 2 * serialization_time);
    EXPECT_EQ(used->queue_wait_time, serialization_time);
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
