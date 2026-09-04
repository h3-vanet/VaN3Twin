/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
#ifndef NR_SL_BEACON_COVERAGE_H
#define NR_SL_BEACON_COVERAGE_H

#include <cstdint>
#include <vector>

namespace ns3 {

/**
 * Coverage-probe bookkeeping for a single designated NR sidelink
 * transmitter, identified by its ns-3 Node ID (e.g. a static beacon or
 * broker node). Disabled by default: until NrSlBeaconCoverageEnable() is
 * called, NrSlBeaconCoverageIsEnabled() returns false and the notify
 * functions are no-ops, so runs that don't use this feature are
 * unaffected.
 *
 * NrSlBeaconCoverageStartNewBeacon() marks the start of a new logical
 * beacon and clears the decoder set. Call it once per application-layer
 * beacon send, NOT from the PHY TX path: with EnableBlindReTx, a single
 * logical beacon produces multiple NrSpectrumPhy::StartTxSlDataFrames
 * calls (initial + up to slMaxTxTransNumPssch-1 blind retransmissions),
 * so resetting there would clear legitimate decodes recorded between
 * copies of the *same* beacon. NrSlBeaconCoverageNotifyDecoded() records
 * a receiving node id as having decoded the current beacon (any one of
 * its copies). NrSlBeaconCoverageGetSnapshot() reads back the current
 * sequence number and the distinct set of node ids that decoded it.
 *
 * That decoder set is *not* pre-filtered to "currently live vehicles":
 * this module has no notion of TraCI/SUMO state (deliberately, to avoid
 * a traci<->nr module dependency — see TraciClient::SetPerTickCallback).
 * ns-3's vehicle-pool nodes (see v2v-emergencyVehicleAlert-nrv2x.cc) all
 * get a fully active sidelink stack up front, whether or not TraCI has
 * claimed them yet or has since released them — an unclaimed or parked
 * node can physically decode a broadcast it's in range of just like a
 * live one. Callers that want "fraction of *live* vehicles" must
 * intersect this set against their own live-vehicle-id set.
 */
void NrSlBeaconCoverageEnable (uint32_t beaconNodeId);
bool NrSlBeaconCoverageIsEnabled (void);
uint32_t NrSlBeaconCoverageGetBeaconNodeId (void);

void NrSlBeaconCoverageStartNewBeacon (void);
void NrSlBeaconCoverageNotifyDecoded (uint32_t rxNodeId);

struct NrSlBeaconCoverageSnapshot
{
  uint64_t seq {0};                    //!< Sequence number of the most recent beacon transmission
  std::vector<uint32_t> decoderNodeIds; //!< Distinct receiving node ids that decoded that sequence (unfiltered — see class comment)
};

NrSlBeaconCoverageSnapshot NrSlBeaconCoverageGetSnapshot (void);

} // namespace ns3

#endif /* NR_SL_BEACON_COVERAGE_H */
