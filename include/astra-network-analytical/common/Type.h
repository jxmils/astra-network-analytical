/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#pragma once

#include <cstdint>

namespace NetworkAnalytical {

/// Callback function pointer: "void func(void*)"
using Callback = void (*)(void*);

/// Callback function argument: void*
using CallbackArg = void*;

/// Device ID which starts from 0
using DeviceId = int;

/// Chunk size in Bytes
using ChunkSize = uint64_t;

/// Bandwidth in GB/s
using Bandwidth = double;

/// Latency in ns
using Latency = double;

/// Event time in ns
using EventTime = uint64_t;

/// Basic multi-dimensional topology building blocks
enum class TopologyBuildingBlock {
    Undefined,
    Ring,
    FullyConnected,
    Switch,
    Mesh2D,
    Torus2D,
    Mesh3D,
    Torus3D,
    Mesh3D3D,
    Torus3D3D,
    Mesh2DSnake,
    Torus2DSnake,
    MeshRowRing,
    MeshRowRingAdaptive,
    MeshSwitch,
    MeshSwitchAdaptive,
    MeshSwitchDirectPreferred,
    MeshSwitchOfflineOracle,
    TorusSwitchDirectOnly,
    TorusSwitchSwitchOnly,
    TorusSwitchDirectOnly2D,
    TorusSwitchSwitchOnly2D,
    TorusSwitchAdaptive,
    TorusSwitchAdaptive2D,
    MultiSwitch6Adaptive,
    OcsSwitch6,
    IdealFlex6R4,
    TorusOcsStatic2D,
    TorusOcsAdaptive2D,
    TorusOcsDirectPreferred2D,
    RingOcsDirectPreferred1D,
    TorusOcsQtp,
    TorusStaticCompletion2D,
    HierarchicalCluster
};

}  // namespace NetworkAnalytical
