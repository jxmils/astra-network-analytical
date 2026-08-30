/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "common/EventQueue.h"
#include "common/NetworkFunction.h"
#include "common/NetworkParser.h"
#include "common/Type.h"
#include "congestion_aware/Chunk.h"
#include "congestion_aware/FileGraph.h"
#include "congestion_aware/Helper.h"
#include "congestion_aware/Hybrid2D.h"
#include "congestion_aware/HierarchicalCluster.h"
#include "congestion_aware/IdealFlex.h"
#include "congestion_aware/Mesh2D.h"
#include "congestion_aware/Mesh3D.h"
#include "congestion_aware/MultiPlaneSwitch.h"
#include "congestion_aware/OcsSwitch.h"
#include "congestion_aware/StaticCompletion.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <numeric>
#include <set>
#include <vector>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

TEST(NetworkFunction, DecimalGigabytesPerSecondEqualBytesPerNanosecond) {
    EXPECT_DOUBLE_EQ(bw_GBps_to_Bpns(1.0), 1.0);
    EXPECT_DOUBLE_EQ(bw_GBps_to_Bpns(200.0), 200.0);
}

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
  "version": 5,
  "endpoints": 16,
  "planes": 6,
  "link_bandwidth_GBps": 1.0,
  "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0,
  "initial_reconfiguration": )" << (initial_reconfiguration ? "true" : "false") << R"(,
  "assignments": [
    {"source": 0, "destination": 1, "bytes": 100, "stream": 0, "route": "OCS", "stripes": [{"plane": 0, "bytes": 100}]},
    {"source": 2, "destination": 3, "bytes": 100, "stream": 0, "route": "OCS", "stripes": [{"plane": 1, "bytes": 100}]},
    {"source": 0, "destination": 2, "bytes": 100, "stream": 0, "route": "OCS", "stripes": [{"plane": 0, "bytes": 100}]}
  ],
  "rounds": [
    {"index": 0, "configurations": [
      {"plane": 0, "stream": 0, "matching": [[0, 1]], "circuits": [{"source": 0, "destination": 1, "bytes": 100}]},
      {"plane": 1, "stream": 0, "matching": [[2, 3]], "circuits": [{"source": 2, "destination": 3, "bytes": 100}]}
    ]},
    {"index": 1, "configurations": [
      {"plane": 0, "stream": 0, "matching": [[0, 2]], "circuits": [{"source": 0, "destination": 2, "bytes": 100}]}
    ]}
  ]
})";
    output.close();
    return path;
}

std::string write_static_completion_test_plan(
    const std::string& base_fabric = "torus2d") {
    const auto path = "/tmp/analytical-static-completion.json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-static-completion",
  "version": 1,
  "endpoints": 16,
  "base_fabric": ")" << base_fabric << R"(",
  "matchings": [
    [[0, 10], [1, 11], [2, 8], [3, 9],
     [4, 14], [5, 15], [6, 12], [7, 13]],
    [[0, 5], [1, 6], [2, 7], [3, 4],
     [8, 13], [9, 14], [10, 15], [11, 12]]
  ]
})";
    output.close();
    return path;
}

std::string write_file_graph_test_plan(const bool directed = false) {
    const auto path = "/tmp/analytical-file-graph.yml";
    auto output = std::ofstream(path);
    output << "format: panel-physical-graph\n"
           << "version: 1\n"
           << "endpoints: 4\n"
           << "devices: 5\n"
           << "directed: " << (directed ? "true" : "false") << "\n"
           << "edges:\n"
           << "  - {source: 0, destination: 4, class: switch}\n"
           << "  - {source: 4, destination: 1, class: switch}\n"
           << "  - {source: 1, destination: 2, class: direct}\n"
           << "  - {source: 2, destination: 3, class: direct}\n";
    output.close();
    return path;
}

std::string write_hybrid_ocs_test_plan() {
    const auto path = "/tmp/analytical-torus-ocs.json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-ocs-plan", "version": 5, "endpoints": 16, "planes": 2,
  "link_bandwidth_GBps": 1.0, "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0, "initial_reconfiguration": false,
  "assignments": [
    {"source": 0, "destination": 1, "bytes": 100, "stream": 0, "route": "DIRECT"},
    {"source": 0, "destination": 10, "bytes": 100, "stream": 0, "route": "OCS", "stripes": [{"plane": 0, "bytes": 100}]},
    {"source": 0, "destination": 10, "bytes": 100, "stream": 1, "route": "DIRECT"},
    {"source": 0, "destination": 10, "bytes": 100, "stream": 2, "route": "OCS", "stripes": [{"plane": 0, "bytes": 100}]}
  ],
  "rounds": [{"index": 0, "configurations": [
    {"plane": 0, "stream": 0, "matching": [[0, 10]], "circuits": [{"source": 0, "destination": 10, "bytes": 100}]}
  ]}, {"index": 1, "configurations": [
    {"plane": 0, "stream": 2, "matching": [[0, 10]], "circuits": [{"source": 0, "destination": 10, "bytes": 100}]}
  ]}]
})";
    output.close();
    return path;
}

std::string write_ring_ocs_test_plan() {
    const auto path = "/tmp/analytical-ring-ocs.json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-ocs-plan", "version": 5, "endpoints": 16, "planes": 4,
  "logical_dimensions": [4, 4],
  "link_bandwidth_GBps": 1.0, "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0, "initial_reconfiguration": false,
  "assignments": [
    {"source": 0, "destination": 8, "bytes": 100, "stream": 0, "route": "DIRECT"},
    {"source": 1, "destination": 9, "bytes": 100, "stream": 0, "route": "DIRECT"}
  ],
  "rounds": []
})";
    output.close();
    return path;
}

std::string write_hybrid_escape_test_plan() {
    const auto path = "/tmp/analytical-torus-ocs-escape.json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-ocs-plan", "version": 5, "endpoints": 16, "planes": 2,
  "link_bandwidth_GBps": 1.0, "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0, "initial_reconfiguration": false,
  "direct_escape_factor": 1.0,
  "assignments": [
    {"source": 0, "destination": 10, "bytes": 800, "stream": 0,
     "route": "OCS", "allow_direct_escape": true, "target_round": 0,
     "stripes": [{"plane": 0, "bytes": 800}]},
    {"source": 0, "destination": 10, "bytes": 100, "stream": 1,
     "route": "OCS", "allow_direct_escape": true, "target_round": 0,
     "stripes": [{"plane": 0, "bytes": 100}]}
  ],
  "rounds": [{"index": 0, "configurations": [
    {"plane": 0, "stream": -1, "matching": [[0, 10]],
     "circuits": [{"source": 0, "destination": 10, "bytes": 900}]}
  ]}]
})";
    output.close();
    return path;
}

std::string write_striped_ocs_test_plan() {
    const auto path = "/tmp/analytical-striped-ocs.json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-ocs-plan", "version": 5, "endpoints": 16, "planes": 2,
  "logical_dimensions": [4, 4],
  "link_bandwidth_GBps": 1.0, "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0, "initial_reconfiguration": false,
  "assignments": [
    {"source": 0, "destination": 1, "bytes": 100, "stream": 0,
     "route": "OCS", "stripes": [
       {"plane": 0, "bytes": 50}, {"plane": 1, "bytes": 50}]}
  ],
  "rounds": [{"index": 0, "configurations": [
    {"plane": 0, "stream": 0, "matching": [[0, 1]],
     "circuits": [{"source": 0, "destination": 1, "bytes": 50}]},
    {"plane": 1, "stream": 0, "matching": [[0, 1]],
     "circuits": [{"source": 0, "destination": 1, "bytes": 50}]}
  ]}]
})";
    output.close();
    return path;
}

std::string write_causal_ocs_test_plan() {
    const auto path = "/tmp/analytical-causal-ocs.json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-ocs-plan", "version": 5, "endpoints": 16, "planes": 2,
  "link_bandwidth_GBps": 1.0, "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0, "initial_reconfiguration": false,
  "assignments": [
    {"source": 0, "destination": 1, "bytes": 100, "stream": 0, "route": "OCS0", "stripes": [{"plane": 0, "bytes": 100}], "request_id": 0, "not_before_ns": 50},
    {"source": 2, "destination": 3, "bytes": 100, "stream": 0, "route": "OCS1", "stripes": [{"plane": 1, "bytes": 100}], "request_id": 1, "not_before_ns": 0},
    {"source": 4, "destination": 5, "bytes": 100, "stream": 1, "route": "OCS1", "stripes": [{"plane": 1, "bytes": 100}], "request_id": 2, "not_before_ns": 0},
    {"source": 6, "destination": 7, "bytes": 100, "stream": 1, "route": "OCS0", "stripes": [{"plane": 0, "bytes": 100}], "request_id": 3, "not_before_ns": 0}
  ],
  "rounds": [
    {"index": 0, "configurations": [
      {"plane": 0, "stream": 0, "matching": [[0, 1]], "circuits": [{"source": 0, "destination": 1, "bytes": 100}]},
      {"plane": 1, "stream": 0, "matching": [[2, 3], [4, 5]], "circuits": [{"source": 2, "destination": 3, "bytes": 100}]}
    ]},
    {"index": 1, "configurations": [
      {"plane": 0, "stream": 1, "matching": [[6, 7]], "circuits": [{"source": 6, "destination": 7, "bytes": 100}]},
      {"plane": 1, "stream": 1, "matching": [[2, 3], [4, 5]], "circuits": [{"source": 4, "destination": 5, "bytes": 100}]}
    ]}
  ]
})";
    output.close();
    return path;
}

std::string write_qtp_ocs_test_plan() {
    const auto path = "/tmp/analytical-qtp-ocs.json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-ocs-plan", "version": 5, "endpoints": 64, "planes": 2,
  "link_bandwidth_GBps": 1.0, "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0, "initial_reconfiguration": false,
  "assignments": [
)";
    auto first = true;
    for (auto source = 0; source < 64; ++source) {
        for (auto destination = 0; destination < 64; ++destination) {
            if (source == destination) {
                continue;
            }
            output << (first ? "" : ",\n")
                   << "    {\"source\": " << source
                   << ", \"destination\": " << destination
                   << ", \"bytes\": 100, \"stream\": 0, "
                      "\"route\": \"DIRECT\"}";
            first = false;
        }
    }
    output << R"(
  ],
  "rounds": []
})";
    output.close();
    return path;
}

std::string write_synchronized_ocs_test_plan() {
    const auto path = "/tmp/analytical-synchronized-ocs.json";
    auto output = std::ofstream(path);
    output << R"({
  "format": "panel-ocs-plan", "version": 5, "endpoints": 16, "planes": 2,
  "link_bandwidth_GBps": 1.0, "propagation_ns": 10.0,
  "reconfiguration_ns": 7.0, "initial_reconfiguration": false,
  "assignments": [
    {"source": 2, "destination": 3, "bytes": 100, "stream": 0,
     "route": "OCS1", "stripes": [{"plane": 1, "bytes": 100}]},
    {"source": 0, "destination": 1, "bytes": 200, "stream": 1,
     "route": "OCS0", "stripes": [{"plane": 0, "bytes": 200}]},
    {"source": 4, "destination": 5, "bytes": 100, "stream": 2,
     "route": "OCS1", "stripes": [{"plane": 1, "bytes": 100}]}
  ],
  "rounds": [
    {"index": 0, "synchronize": true, "configurations": [
      {"plane": 0, "stream": 0, "matching": [[0, 1]], "circuits": []},
      {"plane": 1, "stream": 0, "matching": [[2, 3]],
       "circuits": [{"source": 2, "destination": 3, "bytes": 100}]}
    ]},
    {"index": 1, "configurations": [
      {"plane": 0, "stream": 1, "matching": [[0, 1]],
       "circuits": [{"source": 0, "destination": 1, "bytes": 200}]}
    ]},
    {"index": 2, "synchronize": true, "configurations": [
      {"plane": 0, "stream": 2, "matching": [[6, 7]], "circuits": []},
      {"plane": 1, "stream": -1, "matching": [[4, 5]],
       "circuits": [{"source": 4, "destination": 5, "bytes": 100}]}
    ]}
  ]
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
    EXPECT_EQ(first.arrival_time, 110);
    EXPECT_EQ(second_round.arrival_time, 217);
    EXPECT_EQ(topology.get_completed_rounds(), 3);
    EXPECT_EQ(topology.get_reconfiguration_count(), 1);
    EXPECT_EQ(topology.get_reconfiguration_time(), 7);
    EXPECT_EQ(topology.get_scheduled_bytes(), 300);
    EXPECT_EQ(topology.get_transmitted_bytes(), 300);
    EXPECT_EQ(topology.get_circuit_wait_time(), 107);
    EXPECT_EQ(topology.get_max_circuit_wait_time(), 107);
    EXPECT_EQ(topology.get_circuit_transmissions(), 3);
    EXPECT_EQ(topology.get_plane_schedule_makespan(), 207);
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

    EXPECT_EQ(observation.arrival_time, 117);
    EXPECT_EQ(topology.get_reconfiguration_count(), 3);
    EXPECT_EQ(topology.get_reconfiguration_time(), 21);
    std::remove(plan.c_str());
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       OcsSwitchStripesOneLogicalTransferAndJoinsCompletion) {
    const auto plan = write_striped_ocs_test_plan();
    auto topology = OcsSwitch(16, 1.0, 5.0, plan, 2);
    auto observation = ArrivalObservation{event_queue};
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 1, 100), record_arrival, &observation));
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    EXPECT_EQ(observation.arrival_time, 60);
    EXPECT_EQ(topology.get_scheduled_bytes(), 100);
    EXPECT_EQ(topology.get_transmitted_bytes(), 100);
    EXPECT_EQ(topology.get_circuit_transmissions(), 2);
    EXPECT_EQ(topology.get_plane_schedule_makespan(), 50);
    EXPECT_EQ(topology.get_max_active_ports(0), 2);
    EXPECT_EQ(topology.get_max_distinct_peers(0), 1);
    const auto routes = topology.get_route_metrics();
    ASSERT_EQ(routes.size(), 1);
    EXPECT_EQ(routes.front().hops, 1);
    EXPECT_EQ(routes.front().physical_edges, 2);
    EXPECT_EQ(routes.front().messages, 1);
    EXPECT_EQ(routes.front().payload_bytes, 100);
    EXPECT_EQ(routes.front().byte_hops, 200);
    std::remove(plan.c_str());
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       IdealFlexUsesOneLogicalHopAndSixPortCapacity) {
    auto topology = IdealFlex(64, 1.0, 5.0);
    auto observation = ArrivalObservation{event_queue};
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 1), record_arrival, &observation));
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    EXPECT_EQ(observation.arrival_time, 28);
    const auto routes = topology.get_route_metrics();
    ASSERT_EQ(routes.size(), 1);
    EXPECT_EQ(routes[0].hops, 1);
    EXPECT_EQ(routes[0].physical_edges, 2);
    EXPECT_EQ(routes[0].payload_bytes, 100);
    EXPECT_EQ(routes[0].byte_hops, 200);
    const auto links = topology.get_link_metrics();
    EXPECT_EQ(count_links(links, LinkClass::SwitchUplink), 12 * 64);
    const auto physical_bytes = std::accumulate(
        links.begin(), links.end(), uint64_t{0},
        [](const uint64_t total, const LinkMetrics& metric) {
            return total + metric.bytes;
        });
    EXPECT_EQ(physical_bytes, 200);
}

TEST_F(TestNetworkAnalyticalCongestionAware, TorusOcsUsesDirectAndCircuitPorts) {
    const auto plan = write_hybrid_ocs_test_plan();
    auto topology = OcsSwitch(16, 1.0, 5.0, plan, 2, true);
    auto direct = ArrivalObservation{event_queue};
    auto circuit = ArrivalObservation{event_queue};
    auto later_stream = ArrivalObservation{event_queue};
    EXPECT_EQ(route_link_classes(topology.route(0, 10, 100, 1)),
              std::vector({LinkClass::BaseMesh, LinkClass::BaseMesh,
                           LinkClass::BaseMesh, LinkClass::BaseMesh}));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 10, 100, 2), 2, record_arrival, &later_stream));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 1, 100), record_arrival, &direct));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(0, 10, 100), record_arrival, &circuit));
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    EXPECT_EQ(direct.arrival_time, 105);
    EXPECT_EQ(circuit.arrival_time, 110);
    EXPECT_EQ(later_stream.arrival_time, 210);
    EXPECT_EQ(topology.get_scheduled_bytes(), 200);
    EXPECT_EQ(topology.get_transmitted_bytes(), 200);
    EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::BaseMesh), 64);
    EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::SwitchUplink), 64);
    std::remove(plan.c_str());
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       RingOcsUsesTwoPersistentAndFourOpticalPorts) {
    const auto plan = write_ring_ocs_test_plan();
    auto topology = OcsSwitch(16, 1.0, 5.0, plan, 4, false, false, true);

    EXPECT_EQ(topology.get_npus_count_per_dim(), std::vector<int>({4, 4}));
    EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::BaseMesh), 32);
    EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::SwitchUplink),
              128);
    const auto direct_route = topology.route(0, 8, 100, 0);
    EXPECT_EQ(route_link_classes(direct_route),
              std::vector<LinkClass>(8, LinkClass::BaseMesh));
    auto direct = ArrivalObservation{event_queue};
    topology.send(std::make_unique<Chunk>(
        100, direct_route, 0, record_arrival, &direct));
    while (!event_queue->finished()) {
        event_queue->proceed();
    }
    const auto metrics = topology.get_link_metrics();
    const auto base_bytes = std::accumulate(
        metrics.begin(), metrics.end(), uint64_t{0},
        [](const uint64_t total, const LinkMetrics& metric) {
            return total + (metric.link_class == LinkClass::BaseMesh
                                ? metric.bytes : 0);
        });
    EXPECT_EQ(base_bytes, 800);
    EXPECT_EQ(route_ids(topology.route(1, 9, 100, 0)),
              std::vector<DeviceId>({1, 0, 15, 14, 13, 12, 11, 10, 9}));
    for (auto endpoint = 0; endpoint < 16; ++endpoint) {
        EXPECT_EQ(topology.route(endpoint, endpoint).front()
                      ->get_connected_device_ids().size(), 6);
    }
    std::remove(plan.c_str());
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       PersistentOcsEscapesARequestAndConsumesItsCircuitQuota) {
    const auto plan = write_hybrid_escape_test_plan();
    auto topology = OcsSwitch(16, 1.0, 5.0, plan, 2, true);
    auto optical = ArrivalObservation{event_queue};
    auto escaped = ArrivalObservation{event_queue};

    const auto optical_route = topology.route(0, 10, 800, 0);
    EXPECT_EQ(route_link_classes(optical_route),
              std::vector<LinkClass>(2, LinkClass::SwitchUplink));
    topology.send(std::make_unique<Chunk>(
        800, optical_route, 0, record_arrival, &optical));

    const auto escaped_route = topology.route(0, 10, 100, 1);
    EXPECT_EQ(route_link_classes(escaped_route),
              std::vector<LinkClass>(4, LinkClass::BaseMesh));
    topology.send(std::make_unique<Chunk>(
        100, escaped_route, 1, record_arrival, &escaped));

    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    EXPECT_EQ(topology.get_scheduled_bytes(), 900);
    EXPECT_EQ(topology.get_transmitted_bytes(), 800);
    EXPECT_EQ(topology.get_escaped_bytes(), 100);
    EXPECT_EQ(topology.get_escaped_assignments(), 1);
    EXPECT_EQ(topology.get_completed_rounds(), 1);
    EXPECT_LT(escaped.arrival_time, optical.arrival_time);
    std::remove(plan.c_str());
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       OcsPlanesAdvanceIndependentlyAndHonorCausalAssignments) {
    const auto plan = write_causal_ocs_test_plan();
    auto topology = OcsSwitch(16, 1.0, 5.0, plan, 2, true);
    auto delayed_plane_zero = ArrivalObservation{event_queue};
    auto first_plane_one = ArrivalObservation{event_queue};
    auto second_plane_one = ArrivalObservation{event_queue};
    auto second_plane_zero = ArrivalObservation{event_queue};

    const auto delayed_route = topology.route(0, 1, 100, 0);
    const auto first_plane_one_route = topology.route(2, 3, 100, 0);
    EXPECT_EQ(route_ids(delayed_route), std::vector<DeviceId>({0, 16, 1}));
    EXPECT_EQ(route_ids(first_plane_one_route), std::vector<DeviceId>({2, 17, 3}));
    topology.send(std::make_unique<Chunk>(
        100, delayed_route, 0, record_arrival, &delayed_plane_zero));
    topology.send(std::make_unique<Chunk>(
        100, first_plane_one_route, 0, record_arrival, &first_plane_one));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(4, 5, 100, 1), 1,
        record_arrival, &second_plane_one));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(6, 7, 100, 1), 1,
        record_arrival, &second_plane_zero));
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    EXPECT_GE(delayed_plane_zero.arrival_time, 50);
    EXPECT_LT(second_plane_one.arrival_time, second_plane_zero.arrival_time);
    EXPECT_EQ(topology.get_completed_rounds(), 4);
    EXPECT_EQ(topology.get_reconfiguration_count(), 1);
    EXPECT_EQ(topology.get_reconfiguration_time(), 7);
    std::remove(plan.c_str());
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       OcsSynchronizedRoundWaitsAndInstallsIdlePlane) {
    const auto plan = write_synchronized_ocs_test_plan();
    auto topology = OcsSwitch(16, 1.0, 5.0, plan, 2, true);
    auto first_plane_one = ArrivalObservation{event_queue};
    auto long_plane_zero = ArrivalObservation{event_queue};
    auto second_plane_one = ArrivalObservation{event_queue};

    topology.send(std::make_unique<Chunk>(
        100, topology.route(2, 3, 100, 0), 0,
        record_arrival, &first_plane_one));
    topology.send(std::make_unique<Chunk>(
        200, topology.route(0, 1, 200, 1), 1,
        record_arrival, &long_plane_zero));
    topology.send(std::make_unique<Chunk>(
        100, topology.route(4, 5, 100, 2), 2,
        record_arrival, &second_plane_one));
    while (!event_queue->finished()) {
        event_queue->proceed();
    }

    EXPECT_EQ(first_plane_one.arrival_time, 110);
    EXPECT_EQ(long_plane_zero.arrival_time, 210);
    EXPECT_EQ(second_plane_one.arrival_time, 317);
    EXPECT_GT(second_plane_one.arrival_time, long_plane_zero.arrival_time);
    EXPECT_EQ(topology.get_completed_rounds(), 5);
    EXPECT_EQ(topology.get_reconfiguration_count(), 2);
    EXPECT_EQ(topology.get_reconfiguration_time(), 14);
    EXPECT_EQ(topology.get_max_plane_reconfiguration_time(), 7);
    EXPECT_EQ(topology.get_scheduled_bytes(), 400);
    EXPECT_EQ(topology.get_transmitted_bytes(), 400);
    std::remove(plan.c_str());
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       QtpExposesFourLogicalDimensionsOnTheSameSixPhysicalPorts) {
    const auto plan = write_qtp_ocs_test_plan();
    auto topology = OcsSwitch(64, 1.0, 5.0, plan, 2, true, true);
    EXPECT_EQ(topology.get_npus_count_per_dim(),
              std::vector<int>({4, 2, 4, 2}));
    EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::BaseMesh), 4 * 64);
    EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::SwitchUplink),
              4 * 64);

    const auto& placement = topology.get_logical_to_physical();
    EXPECT_EQ(placement.size(), 64);
    EXPECT_EQ(std::set<int>(placement.begin(), placement.end()).size(), 64);
    for (auto source = 0; source < 64; ++source) {
        EXPECT_EQ(topology.route(source, source).front()
                      ->get_connected_device_ids().size(), 6);
        for (auto destination = 0; destination < 64; ++destination) {
            if (source == destination) {
                continue;
            }
            const auto path = topology.route(source, destination, 100, 0);
            const auto classes = route_link_classes(path);
            EXPECT_TRUE(std::all_of(classes.begin(), classes.end(),
                                    [](const LinkClass value) {
                                        return value == LinkClass::BaseMesh;
                                    }));
            const auto source_physical = placement[source];
            const auto destination_physical = placement[destination];
            const auto x_distance = std::abs(source_physical % 8 -
                                             destination_physical % 8);
            const auto y_distance = std::abs(source_physical / 8 -
                                             destination_physical / 8);
            const auto expected_hops = std::min(x_distance, 8 - x_distance) +
                                       std::min(y_distance, 8 - y_distance);
            EXPECT_EQ(path.size(), expected_hops + 1);
            auto previous = path.begin();
            auto following = std::next(previous);
            while (following != path.end()) {
                const auto first_physical = placement[previous->device->get_id()];
                const auto second_physical = placement[following->device->get_id()];
                const auto dx = std::abs(first_physical % 8 - second_physical % 8);
                const auto dy = std::abs(first_physical / 8 - second_physical / 8);
                EXPECT_TRUE((dx == 1 && dy == 0) || (dx == 7 && dy == 0) ||
                            (dx == 0 && dy == 1) || (dx == 0 && dy == 7));
                ++previous;
                ++following;
            }
        }
    }
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

TEST_F(TestNetworkAnalyticalCongestionAware,
       StaticCompletionHasExactPortsAndTransparentOpticalEdges) {
    const auto plan = write_static_completion_test_plan();
    const auto topology = StaticCompletion(16, 1.0, 5.0, 1.0, 5.0, plan);
    const auto metrics = topology.get_link_metrics();

    EXPECT_EQ(count_links(metrics, LinkClass::BaseMesh), 4 * 16);
    EXPECT_EQ(count_links(metrics, LinkClass::SwitchUplink), 2 * 16);
    EXPECT_EQ(topology.get_npus_count_per_dim(), std::vector<int>({4, 4}));
    EXPECT_EQ(topology.get_bandwidth_per_dim(),
              std::vector<Bandwidth>({1.0, 1.0}));
    for (auto endpoint = 0; endpoint < 16; ++endpoint) {
        const auto device = topology.route(endpoint, endpoint).front();
        EXPECT_EQ(device->get_connected_device_ids().size(), 6);
    }

    const auto optical_route = topology.route(0, 10, 100);
    EXPECT_EQ(route_ids(optical_route), std::vector<DeviceId>({0, 10}));
    EXPECT_EQ(route_link_classes(optical_route),
              std::vector<LinkClass>(1, LinkClass::SwitchUplink));
    EXPECT_EQ(std::remove(plan.c_str()), 0);
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       FileGraphLoadsSwitchesAndRoutesDeterministically) {
    const auto plan = write_file_graph_test_plan();
    const auto topology = FileGraph(4, 200.0, 1'000.0, plan);
    EXPECT_EQ(topology.get_npus_count(), 4);
    EXPECT_EQ(topology.get_devices_count(), 5);
    EXPECT_EQ(route_ids(topology.route(0, 3, 1'024)),
              std::vector<DeviceId>({0, 4, 1, 2, 3}));
    EXPECT_EQ(route_link_classes(topology.route(0, 2, 1'024)),
              std::vector<LinkClass>({LinkClass::SwitchUplink,
                                      LinkClass::SwitchUplink,
                                      LinkClass::BaseMesh}));
    EXPECT_EQ(route_ids(topology.route(3, 0, 1'024)),
              std::vector<DeviceId>({3, 2, 1, 4, 0}));
    EXPECT_EQ(std::remove(plan.c_str()), 0);
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       FileGraphPreservesDirectedEdges) {
    const auto plan = write_file_graph_test_plan(true);
    const auto topology = FileGraph(4, 200.0, 1'000.0, plan);
    EXPECT_EQ(route_ids(topology.route(0, 3, 1'024)),
              std::vector<DeviceId>({0, 4, 1, 2, 3}));
    EXPECT_EQ(count_links(topology.get_link_metrics(), LinkClass::SwitchUplink),
              2);
    EXPECT_EQ(std::remove(plan.c_str()), 0);
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       RowRingStaticCompletionUsesFourEndpointPorts) {
    const auto plan = write_static_completion_test_plan("row_rings");
    const auto topology = StaticCompletion(
        16, 1.0, 5.0, 1.0, 5.0, plan,
        StaticCompletion::BaseFabric::RowRings);
    const auto metrics = topology.get_link_metrics();

    EXPECT_EQ(count_links(metrics, LinkClass::BaseMesh), 2 * 16);
    EXPECT_EQ(count_links(metrics, LinkClass::SwitchUplink), 2 * 16);
    for (auto endpoint = 0; endpoint < 16; ++endpoint) {
        const auto device = topology.route(endpoint, endpoint).front();
        EXPECT_EQ(device->get_connected_device_ids().size(), 4);
    }
    EXPECT_EQ(route_ids(topology.route(0, 10, 100)),
              std::vector<DeviceId>({0, 10}));
    EXPECT_EQ(std::remove(plan.c_str()), 0);
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       StaticCompletionRoutesAreValidAndDeterministic) {
    const auto plan = write_static_completion_test_plan();
    const auto topology = StaticCompletion(
        16, 200.0, 1'000.0, 200.0, 1'000.0, plan);
    for (auto source = 0; source < 16; ++source) {
        for (auto destination = 0; destination < 16; ++destination) {
            const auto first = topology.route(source, destination, chunk_size);
            const auto second = topology.route(source, destination, chunk_size);
            EXPECT_EQ(route_ids(first), route_ids(second));
            EXPECT_EQ(route_ids(first).front(), source);
            EXPECT_EQ(route_ids(first).back(), destination);
            route_link_classes(first);
        }
    }
    EXPECT_EQ(std::remove(plan.c_str()), 0);
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       StaticCompletionChargesOneOpticalPathLatency) {
    const auto measure = [this](const Latency path_latency) {
        event_queue = std::make_shared<EventQueue>();
        Topology::set_event_queue(event_queue);
        const auto plan = write_static_completion_test_plan();
        auto topology = StaticCompletion(16, 1.0, 0.0, 1.0, path_latency, plan);
        auto observation = ArrivalObservation{event_queue};
        topology.send(std::make_unique<Chunk>(
            100, topology.route(0, 10, 100), record_arrival, &observation));
        while (!event_queue->finished()) {
            event_queue->proceed();
        }
        EXPECT_EQ(std::remove(plan.c_str()), 0);
        return observation.arrival_time;
    };

    EXPECT_EQ(measure(5.0) - measure(0.0), 5);
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

TEST_F(TestNetworkAnalyticalCongestionAware, ExplicitRouteUsesRequestStream) {
    const auto path = std::string("/tmp/astra-explicit-route-test.tsv");
    {
        auto output = std::ofstream(path);
        output << "123\t0\t5\t0,4,5\n";
    }
    ASSERT_EQ(setenv("ASTRA_EXPLICIT_ROUTE_FILE", path.c_str(), 1), 0);
    const auto topology = Mesh3D(
        std::array<int, 3>{4, 4, 4}, 200.0, 1'000.0, false);
    ASSERT_EQ(unsetenv("ASTRA_EXPLICIT_ROUTE_FILE"), 0);
    std::remove(path.c_str());

    const Topology& routed = topology;
    EXPECT_EQ(route_ids(routed.route(0, 5, 1'024, 123)),
              std::vector<DeviceId>({0, 4, 5}));
    EXPECT_EQ(route_ids(routed.route(0, 5, 1'024, 124)),
              std::vector<DeviceId>({0, 1, 5}));
}

TEST_F(TestNetworkAnalyticalCongestionAware,
       RectangularThreeDimensionalTorusHasSixDistinctPortsAndMinimalRoutes) {
    constexpr auto extents = std::array<int, 3>{4, 8, 8};
    constexpr auto npus = extents[0] * extents[1] * extents[2];
    const auto topology = Mesh3D(extents, 200.0, 1'000.0, true, true);

    EXPECT_EQ(topology.get_npus_count_per_dim(),
              std::vector<int>({4, 8, 8}));
    for (auto source = 0; source < npus; ++source) {
        const auto neighbors = topology.route(source, source).front()
                                   ->get_connected_device_ids();
        EXPECT_EQ(neighbors.size(), 6);
        EXPECT_EQ(std::set<DeviceId>(neighbors.begin(), neighbors.end()).size(), 6);
        for (auto destination = 0; destination < npus; ++destination) {
            const auto ids = route_ids(topology.route(source, destination));
            auto expected_hops = 0;
            auto stride = 1;
            for (const auto extent : extents) {
                const auto first = (source / stride) % extent;
                const auto second = (destination / stride) % extent;
                const auto distance = std::abs(first - second);
                expected_hops += std::min(distance, extent - distance);
                stride *= extent;
            }
            EXPECT_EQ(ids.size() - 1,
                      static_cast<std::size_t>(expected_hops));
        }
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
       TopologyAwareFullSwitchChangesOnlyLogicalDimensions) {
    const auto ordinary = MultiPlaneSwitch(64, 200.0, 1'000.0);
    const auto grouped = MultiPlaneSwitch(64, 200.0, 1'000.0, true);
    EXPECT_EQ(ordinary.get_npus_count_per_dim(), std::vector<int>({64}));
    EXPECT_EQ(grouped.get_npus_count_per_dim(),
              std::vector<int>({4, 4, 4}));
    EXPECT_EQ(ordinary.get_link_metrics().size(),
              grouped.get_link_metrics().size());
    EXPECT_EQ(route_ids(ordinary.route(0, 63)),
              route_ids(grouped.route(0, 63)));
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
    EXPECT_EQ(simulation_time, 64'413);
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
    EXPECT_EQ(simulation_time, 21'471);
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
    EXPECT_EQ(simulation_time, 42'942);
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
    EXPECT_EQ(simulation_time, 755'956);
}
