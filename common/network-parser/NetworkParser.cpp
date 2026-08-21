/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "common/NetworkParser.h"
#include <cassert>
#include <iostream>

using namespace NetworkAnalytical;

NetworkParser::NetworkParser(const std::string& path) noexcept
    : dims_count(-1), direct_preference_factor(1.10) {
    // initialize values
    npus_count_per_dim = {};
    bandwidth_per_dim = {};
    latency_per_dim = {};
    extra_bandwidth_per_dim = {};
    extra_latency_per_dim = {};
    topology_per_dim = {};

    try {
        // load network config file
        const auto network_config = YAML::LoadFile(path);

        // parse network configs
        parse_network_config_yml(network_config);
    } catch (const YAML::BadFile& e) {
        // loading network config file failed
        std::cerr << "[Error] (network/analytical) " << e.what() << std::endl;
        std::exit(-1);
    }
}

int NetworkParser::get_dims_count() const noexcept {
    assert(dims_count > 0);

    return dims_count;
}

std::vector<int> NetworkParser::get_npus_counts_per_dim() const noexcept {
    assert(dims_count > 0);
    assert(npus_count_per_dim.size() == dims_count);

    return npus_count_per_dim;
}

std::vector<Bandwidth> NetworkParser::get_bandwidths_per_dim() const noexcept {
    assert(dims_count > 0);
    assert(bandwidth_per_dim.size() == dims_count);

    return bandwidth_per_dim;
}

std::vector<Latency> NetworkParser::get_latencies_per_dim() const noexcept {
    assert(dims_count > 0);
    assert(latency_per_dim.size() == dims_count);

    return latency_per_dim;
}

std::vector<Bandwidth> NetworkParser::get_extra_bandwidths_per_dim() const noexcept {
    assert(extra_bandwidth_per_dim.size() == dims_count);
    return extra_bandwidth_per_dim;
}

std::vector<Latency> NetworkParser::get_extra_latencies_per_dim() const noexcept {
    assert(extra_latency_per_dim.size() == dims_count);
    return extra_latency_per_dim;
}

double NetworkParser::get_direct_preference_factor() const noexcept {
    return direct_preference_factor;
}

const std::string& NetworkParser::get_routing_plan_path() const noexcept {
    return routing_plan_path;
}

std::vector<TopologyBuildingBlock> NetworkParser::get_topologies_per_dim() const noexcept {
    assert(dims_count > 0);
    assert(topology_per_dim.size() == dims_count);

    return topology_per_dim;
}

void NetworkParser::parse_network_config_yml(const YAML::Node& network_config) noexcept {
    // parse topology_per_dim
    const auto topology_names = parse_vector<std::string>(network_config["topology"]);
    for (const auto& topology_name : topology_names) {
        const auto topology_dim = NetworkParser::parse_topology_name(topology_name);
        topology_per_dim.push_back(topology_dim);
    }

    // set dims_count
    dims_count = static_cast<int>(topology_per_dim.size());

    // parse vector values
    npus_count_per_dim = parse_vector<int>(network_config["npus_count"]);
    bandwidth_per_dim = parse_vector<Bandwidth>(network_config["bandwidth"]);
    latency_per_dim = parse_vector<Latency>(network_config["latency"]);
    extra_bandwidth_per_dim = network_config["extra_bandwidth"]
                                  ? parse_vector<Bandwidth>(network_config["extra_bandwidth"])
                                  : bandwidth_per_dim;
    extra_latency_per_dim = network_config["extra_latency"]
                                ? parse_vector<Latency>(network_config["extra_latency"])
                                : latency_per_dim;
    direct_preference_factor = network_config["direct_preference_factor"]
                                   ? network_config["direct_preference_factor"].as<double>()
                                   : 1.10;
    routing_plan_path = network_config["routing_plan"]
                            ? network_config["routing_plan"].as<std::string>()
                            : "";

    // check the validity of the parsed network config
    check_validity();
}

TopologyBuildingBlock NetworkParser::parse_topology_name(const std::string& topology_name) noexcept {
    assert(!topology_name.empty());

    if (topology_name == "Ring") {
        return TopologyBuildingBlock::Ring;
    }

    if (topology_name == "FullyConnected") {
        return TopologyBuildingBlock::FullyConnected;
    }

    if (topology_name == "Mesh2DSnake") {
        return TopologyBuildingBlock::Mesh2DSnake;
    }

    if (topology_name == "Torus2DSnake") {
        return TopologyBuildingBlock::Torus2DSnake;
    }

    if (topology_name == "Mesh2D") {
        return TopologyBuildingBlock::Mesh2D;
    }

    if (topology_name == "Torus2D") {
        return TopologyBuildingBlock::Torus2D;
    }

    if (topology_name == "Mesh3D") {
        return TopologyBuildingBlock::Mesh3D;
    }

    if (topology_name == "Torus3D") {
        return TopologyBuildingBlock::Torus3D;
    }

    if (topology_name == "MeshRowRing") {
        return TopologyBuildingBlock::MeshRowRing;
    }

    if (topology_name == "MeshRowRingAdaptive") {
        return TopologyBuildingBlock::MeshRowRingAdaptive;
    }

    if (topology_name == "MeshSwitch") {
        return TopologyBuildingBlock::MeshSwitch;
    }

    if (topology_name == "MeshSwitchAdaptive") {
        return TopologyBuildingBlock::MeshSwitchAdaptive;
    }

    if (topology_name == "MeshSwitchDirectPreferred") {
        return TopologyBuildingBlock::MeshSwitchDirectPreferred;
    }

    if (topology_name == "MeshSwitchOfflineOracle") {
        return TopologyBuildingBlock::MeshSwitchOfflineOracle;
    }

    if (topology_name == "TorusSwitchAdaptive") {
        return TopologyBuildingBlock::TorusSwitchAdaptive;
    }

    if (topology_name == "TorusSwitchDirectOnly") {
        return TopologyBuildingBlock::TorusSwitchDirectOnly;
    }

    if (topology_name == "TorusSwitchSwitchOnly") {
        return TopologyBuildingBlock::TorusSwitchSwitchOnly;
    }

    if (topology_name == "MultiSwitch6Adaptive") {
        return TopologyBuildingBlock::MultiSwitch6Adaptive;
    }

    if (topology_name == "Switch") {
        return TopologyBuildingBlock::Switch;
    }

    // shouldn't reach here
    std::cerr << "[Error] (network/analytical) " << "Topology name " << topology_name << " not supported" << std::endl;
    std::exit(-1);
}

void NetworkParser::check_validity() const noexcept {
    // dims_count should match
    if (dims_count != npus_count_per_dim.size()) {
        std::cerr << "[Error] (network/analytical) " << "length of npus_count (" << npus_count_per_dim.size()
                  << ") doesn't match with dimensions (" << dims_count << ")" << std::endl;
        std::exit(-1);
    }

    if (dims_count != bandwidth_per_dim.size()) {
        std::cerr << "[Error] (network/analytical) " << "length of bandwidth (" << bandwidth_per_dim.size()
                  << ") doesn't match with dims_count (" << dims_count << ")" << std::endl;
        std::exit(-1);
    }

    if (dims_count != latency_per_dim.size()) {
        std::cerr << "[Error] (network/analytical) " << "length of latency (" << latency_per_dim.size()
                  << ") doesn't match with dims_count (" << dims_count << ")" << std::endl;
        std::exit(-1);
    }


    if (dims_count != extra_bandwidth_per_dim.size()) {
        std::cerr << "[Error] (network/analytical) length of extra_bandwidth ("
                  << extra_bandwidth_per_dim.size()
                  << ") doesn't match with dims_count (" << dims_count << ")" << std::endl;
        std::exit(-1);
    }

    if (dims_count != extra_latency_per_dim.size()) {
        std::cerr << "[Error] (network/analytical) length of extra_latency ("
                  << extra_latency_per_dim.size()
                  << ") doesn't match with dims_count (" << dims_count << ")" << std::endl;
        std::exit(-1);
    }

    if (direct_preference_factor < 1.0) {
        std::cerr << "[Error] (network/analytical) direct_preference_factor ("
                  << direct_preference_factor << ") should be at least 1" << std::endl;
        std::exit(-1);
    }

    // npus_count should be all positive
    for (const auto& npus_count : npus_count_per_dim) {
        if (npus_count <= 1) {
            std::cerr << "[Error] (network/analytical) " << "npus_count (" << npus_count << ") should be larger than 1"
                      << std::endl;
            std::exit(-1);
        }
    }

    // bandwidths should be all positive
    for (const auto& bandwidth : bandwidth_per_dim) {
        if (bandwidth <= 0) {
            std::cerr << "[Error] (network/analytical) " << "bandwidth (" << bandwidth << ") should be larger than 0"
                      << std::endl;
            std::exit(-1);
        }
    }

    for (const auto& bandwidth : extra_bandwidth_per_dim) {
        if (bandwidth <= 0) {
            std::cerr << "[Error] (network/analytical) extra bandwidth (" << bandwidth
                      << ") should be larger than 0" << std::endl;
            std::exit(-1);
        }
    }

    // latency should be non-negative
    for (const auto& latency : latency_per_dim) {
        if (latency < 0) {
            std::cerr << "[Error] (network/analytical) " << "latency (" << latency << ") should be non-negative"
                      << std::endl;
            std::exit(-1);
        }
    }

    for (const auto& latency : extra_latency_per_dim) {
        if (latency < 0) {
            std::cerr << "[Error] (network/analytical) extra latency (" << latency
                      << ") should be non-negative" << std::endl;
            std::exit(-1);
        }
    }
}
