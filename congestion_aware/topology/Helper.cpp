/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "congestion_aware/Helper.h"
#include "congestion_aware/FullyConnected.h"
#include "congestion_aware/Hybrid2D.h"
#include "congestion_aware/HierarchicalCluster.h"
#include "congestion_aware/IdealFlex.h"
#include "congestion_aware/Mesh2D.h"
#include "congestion_aware/Mesh3D.h"
#include "congestion_aware/MultiPlaneSwitch.h"
#include "congestion_aware/OcsSwitch.h"
#include "congestion_aware/Ring.h"
#include "congestion_aware/Switch.h"
#include <cstdlib>
#include <iostream>

using namespace NetworkAnalytical;
using namespace NetworkAnalyticalCongestionAware;

std::shared_ptr<Topology> NetworkAnalyticalCongestionAware::construct_topology(
    const NetworkParser& network_parser) noexcept {
    // get network_parser info
    const auto dims_count = network_parser.get_dims_count();
    const auto topologies_per_dim = network_parser.get_topologies_per_dim();
    const auto npus_counts_per_dim = network_parser.get_npus_counts_per_dim();
    const auto bandwidths_per_dim = network_parser.get_bandwidths_per_dim();
    const auto latencies_per_dim = network_parser.get_latencies_per_dim();
    const auto extra_bandwidths_per_dim = network_parser.get_extra_bandwidths_per_dim();
    const auto extra_latencies_per_dim = network_parser.get_extra_latencies_per_dim();
    const auto direct_preference_factor = network_parser.get_direct_preference_factor();
    const auto nic_count = network_parser.get_nic_count();
    const auto& routing_plan_path = network_parser.get_routing_plan_path();
    const auto& ocs_plan_path = network_parser.get_ocs_plan_path();

    // for now, congestion_aware backend supports 1-dim topology only
    if (dims_count != 1) {
        std::cerr << "[Error] (network/analytical/congestion_aware) " << "only support 1-dim topology" << std::endl;
        std::exit(-1);
    }

    // retrieve basic basic-topology info
    const auto topology_type = topologies_per_dim[0];
    const auto npus_count = npus_counts_per_dim[0];
    const auto bandwidth = bandwidths_per_dim[0];
    const auto latency = latencies_per_dim[0];
    const auto extra_bandwidth = extra_bandwidths_per_dim[0];
    const auto extra_latency = extra_latencies_per_dim[0];

    switch (topology_type) {
    case TopologyBuildingBlock::Ring:
        return std::make_shared<Ring>(npus_count, bandwidth, latency);
    case TopologyBuildingBlock::Switch:
        return std::make_shared<Switch>(npus_count, bandwidth, latency);
    case TopologyBuildingBlock::Mesh2D:
        return std::make_shared<Mesh2D>(npus_count, bandwidth, latency, false);
    case TopologyBuildingBlock::Torus2D:
        return std::make_shared<Mesh2D>(npus_count, bandwidth, latency, true);
    case TopologyBuildingBlock::Mesh3D:
        return std::make_shared<Mesh3D>(npus_count, bandwidth, latency, false);
    case TopologyBuildingBlock::Torus3D:
        return std::make_shared<Mesh3D>(npus_count, bandwidth, latency, true);
    case TopologyBuildingBlock::Mesh3D3D:
        return std::make_shared<Mesh3D>(npus_count, bandwidth, latency, false, true);
    case TopologyBuildingBlock::Torus3D3D:
        return std::make_shared<Mesh3D>(npus_count, bandwidth, latency, true, true);
    case TopologyBuildingBlock::Mesh2DSnake:
        return std::make_shared<Mesh2D>(npus_count, bandwidth, latency, false, Mesh2D::Embedding::Snake);
    case TopologyBuildingBlock::Torus2DSnake:
        return std::make_shared<Mesh2D>(npus_count, bandwidth, latency, true, Mesh2D::Embedding::Snake);
    case TopologyBuildingBlock::MeshRowRing:
        return std::make_shared<Hybrid2D>(npus_count, bandwidth, latency,
                                          Hybrid2D::ExtraFabric::RowRing,
                                          Hybrid2D::RoutingPolicy::Static,
                                          extra_bandwidth, extra_latency);
    case TopologyBuildingBlock::MeshRowRingAdaptive:
        return std::make_shared<Hybrid2D>(npus_count, bandwidth, latency,
                                          Hybrid2D::ExtraFabric::RowRing,
                                          Hybrid2D::RoutingPolicy::Adaptive,
                                          extra_bandwidth, extra_latency);
    case TopologyBuildingBlock::MeshSwitch:
        return std::make_shared<Hybrid2D>(npus_count, bandwidth, latency,
                                          Hybrid2D::ExtraFabric::Switch,
                                          Hybrid2D::RoutingPolicy::Static,
                                          extra_bandwidth, extra_latency);
    case TopologyBuildingBlock::MeshSwitchAdaptive:
        return std::make_shared<Hybrid2D>(npus_count, bandwidth, latency,
                                          Hybrid2D::ExtraFabric::Switch,
                                          Hybrid2D::RoutingPolicy::Adaptive,
                                          extra_bandwidth, extra_latency);
    case TopologyBuildingBlock::MeshSwitchDirectPreferred:
        return std::make_shared<Hybrid2D>(
            npus_count, bandwidth, latency, Hybrid2D::ExtraFabric::Switch,
            Hybrid2D::RoutingPolicy::DirectPreferredAdaptive,
            extra_bandwidth, extra_latency, direct_preference_factor);
    case TopologyBuildingBlock::MeshSwitchOfflineOracle:
        return std::make_shared<Hybrid2D>(
            npus_count, bandwidth, latency, Hybrid2D::ExtraFabric::Switch,
            Hybrid2D::RoutingPolicy::OfflineOracle,
            extra_bandwidth, extra_latency, direct_preference_factor,
            routing_plan_path);
    case TopologyBuildingBlock::TorusSwitchAdaptive:
        return std::make_shared<Hybrid2D>(
            npus_count, bandwidth, latency, Hybrid2D::ExtraFabric::Switch,
            Hybrid2D::RoutingPolicy::Adaptive, extra_bandwidth, extra_latency,
            direct_preference_factor, "", true);
    case TopologyBuildingBlock::TorusSwitchAdaptive2D:
        return std::make_shared<Hybrid2D>(
            npus_count, bandwidth, latency, Hybrid2D::ExtraFabric::Switch,
            Hybrid2D::RoutingPolicy::Adaptive, extra_bandwidth, extra_latency,
            direct_preference_factor, "", true,
            Hybrid2D::LogicalShape::Grid);
    case TopologyBuildingBlock::TorusSwitchDirectOnly:
        return std::make_shared<Hybrid2D>(
            npus_count, bandwidth, latency, Hybrid2D::ExtraFabric::Switch,
            Hybrid2D::RoutingPolicy::DirectOnly, extra_bandwidth, extra_latency,
            direct_preference_factor, "", true);
    case TopologyBuildingBlock::TorusSwitchSwitchOnly:
        return std::make_shared<Hybrid2D>(
            npus_count, bandwidth, latency, Hybrid2D::ExtraFabric::Switch,
            Hybrid2D::RoutingPolicy::SwitchOnly, extra_bandwidth, extra_latency,
            direct_preference_factor, "", true);
    case TopologyBuildingBlock::TorusSwitchDirectOnly2D:
        return std::make_shared<Hybrid2D>(
            npus_count, bandwidth, latency, Hybrid2D::ExtraFabric::Switch,
            Hybrid2D::RoutingPolicy::DirectOnly, extra_bandwidth, extra_latency,
            direct_preference_factor, "", true, Hybrid2D::LogicalShape::Grid);
    case TopologyBuildingBlock::TorusSwitchSwitchOnly2D:
        return std::make_shared<Hybrid2D>(
            npus_count, bandwidth, latency, Hybrid2D::ExtraFabric::Switch,
            Hybrid2D::RoutingPolicy::SwitchOnly, extra_bandwidth, extra_latency,
            direct_preference_factor, "", true, Hybrid2D::LogicalShape::Grid);
    case TopologyBuildingBlock::MultiSwitch6Adaptive:
        return std::make_shared<MultiPlaneSwitch>(npus_count, bandwidth, latency);
    case TopologyBuildingBlock::OcsSwitch6:
        return std::make_shared<OcsSwitch>(npus_count, bandwidth, latency,
                                           ocs_plan_path);
    case TopologyBuildingBlock::IdealFlex6R4:
        return std::make_shared<IdealFlex>(npus_count, bandwidth, latency);
    case TopologyBuildingBlock::TorusOcsStatic2D:
    case TopologyBuildingBlock::TorusOcsAdaptive2D:
    case TopologyBuildingBlock::TorusOcsDirectPreferred2D:
        return std::make_shared<OcsSwitch>(npus_count, bandwidth, latency,
                                           ocs_plan_path, 2, true);
    case TopologyBuildingBlock::TorusOcsQtp:
        return std::make_shared<OcsSwitch>(npus_count, bandwidth, latency,
                                           ocs_plan_path, 2, true, true);
    case TopologyBuildingBlock::HierarchicalCluster:
        return std::make_shared<HierarchicalCluster>(
            npus_count, bandwidth, latency, extra_bandwidth, extra_latency,
            nic_count);
    case TopologyBuildingBlock::FullyConnected:
        return std::make_shared<FullyConnected>(npus_count, bandwidth, latency);
    default:
        // shouldn't reaach here
        std::cerr << "[Error] (network/analytical/congestion_aware) " << "not supported basic-topology" << std::endl;
        std::exit(-1);
    }
}
