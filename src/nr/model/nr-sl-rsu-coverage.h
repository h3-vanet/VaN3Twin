/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
#ifndef NR_SL_RSU_COVERAGE_H
#define NR_SL_RSU_COVERAGE_H

#include <cstdint>
#include <vector>

namespace ns3 {

/**
 * Coverage-probe bookkeeping for a set of RSU (broker-over-sidelink)
 * transmitters, identified by their ns-3 Node IDs. Deliberately a SEPARATE
 * mechanism from NrSlBeaconCoverage (nr-sl-beacon-coverage.h/.cc), not an
 * extension of it, for two reasons:
 *
 *  - NrSlBeaconCoverage is scoped to exactly one transmitter and one
 *    logical-sequence-at-a-time (StartNewBeacon() clears and re-arms on
 *    every application-layer beacon send). It backs 175 already-completed
 *    experimental runs; changing its data layout or call sites risks
 *    changing its behavior for them, which the task explicitly rules out.
 *  - RSU transmissions are event-driven (they fire only when the broker
 *    has something to send, via TraciClient::RegisterRsuSend), not a
 *    periodic 1 Hz send with one clearly-delimited "logical beacon" per
 *    invocation the way --beacon is. There is no equivalent of
 *    StartNewBeacon() to call: instead, this module accumulates
 *    transmit-attempt counts and distinct decoders PER RSU, over
 *    whatever window the caller chooses to sample at (in practice, once
 *    per TraciClient per-tick callback -- see
 *    NrSlRsuCoverageGetSnapshotAndReset()), rather than per logical send.
 *
 * Disabled by default: until NrSlRsuCoverageEnable() is called (only done
 * when --rsu-count > 0), NrSlRsuCoverageIsEnabled() returns false and
 * every notify function is a no-op, so a --rsu-count=0 run is completely
 * unaffected -- including at the PHY call site, which checks
 * IsEnabled()/IsRsuNodeId() before doing anything (see nr-spectrum-phy.cc).
 *
 * "Covered" vs. "not addressed": NrSlRsuCoverageNotifyTx(rsuNodeId) is
 * called once per broker envelope actually sent from that RSU (at the
 * TraciClient::RegisterRsuSend dispatch site, not per PHY-layer blind
 * retransmission -- same reasoning as NrSlBeaconCoverageStartNewBeacon()
 * being called at the app layer, not the PHY TX path). A window in which
 * a given RSU's txCount is 0 means that RSU addressed nobody during the
 * window -- there is no coverage claim to make about it, "not addressed",
 * not "not covered". A window with txCount > 0 for which a live vehicle
 * never appears in decoderNodeIds means it heard nothing decodable from
 * that RSU while the RSU was transmitting -- that vehicle is genuinely
 * "not covered" by that RSU this window. The overall (all-RSU) figure
 * follows the same rule using the sum of per-RSU tx counts.
 */
void NrSlRsuCoverageEnable (const std::vector<uint32_t>& rsuNodeIds);
bool NrSlRsuCoverageIsEnabled (void);
bool NrSlRsuCoverageIsRsuNodeId (uint32_t nodeId);

void NrSlRsuCoverageNotifyTx (uint32_t rsuNodeId);
void NrSlRsuCoverageNotifyDecoded (uint32_t rsuNodeId, uint32_t rxNodeId);

struct NrSlRsuCoverageSnapshot
{
  struct PerRsu
  {
    uint32_t rsuNodeId {0};
    uint32_t txCount {0};                 //!< transmit attempts by this RSU during the window just ended
    std::vector<uint32_t> decoderNodeIds; //!< distinct receiving node ids that decoded >=1 of them
  };
  std::vector<PerRsu> perRsu; //!< one entry per RSU registered via Enable(), in registration order
};

/**
 * Read back accumulated tx/decode counts since the last call (or since
 * Enable(), for the first call), then clear them for the next window.
 * Call once per sampling window (in practice, once per TraciClient
 * per-tick callback firing).
 */
NrSlRsuCoverageSnapshot NrSlRsuCoverageGetSnapshotAndReset (void);

} // namespace ns3

#endif /* NR_SL_RSU_COVERAGE_H */
