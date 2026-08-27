/******************************************************************************
This source code is licensed under the MIT license found in the
LICENSE file in the root directory of this source tree.
*******************************************************************************/

#include "common/NetworkFunction.h"
#include <cassert>

using namespace NetworkAnalytical;

Bandwidth NetworkAnalytical::bw_GBps_to_Bpns(const Bandwidth bw_GBps) noexcept {
    assert(bw_GBps > 0);

    // Decimal SI units: 1 GB/s is exactly 1 B/ns.
    return bw_GBps;
}
