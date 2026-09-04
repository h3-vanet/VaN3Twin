/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
#ifndef NR_SL_BEACON_COVERAGE_H
#define NR_SL_BEACON_COVERAGE_H

#include <cstdint>

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
 * sequence number and its distinct-decoder count.
 */
void NrSlBeaconCoverageEnable (uint32_t beaconNodeId);
bool NrSlBeaconCoverageIsEnabled (void);
uint32_t NrSlBeaconCoverageGetBeaconNodeId (void);

void NrSlBeaconCoverageStartNewBeacon (void);
void NrSlBeaconCoverageNotifyDecoded (uint32_t rxNodeId);

struct NrSlBeaconCoverageSnapshot
{
  uint64_t seq {0};    //!< Sequence number of the most recent beacon transmission
  uint32_t decoded {0}; //!< Distinct receiving node count that decoded that sequence
};

NrSlBeaconCoverageSnapshot NrSlBeaconCoverageGetSnapshot (void);

} // namespace ns3

#endif /* NR_SL_BEACON_COVERAGE_H */
