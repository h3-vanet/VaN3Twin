/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
#include "nr-sl-beacon-coverage.h"

#include <set>

namespace ns3 {

namespace {

bool g_enabled = false;
uint32_t g_beaconNodeId = 0;
uint64_t g_seq = 0;
std::set<uint32_t> g_decoders;

} // namespace

void
NrSlBeaconCoverageEnable (uint32_t beaconNodeId)
{
  g_enabled = true;
  g_beaconNodeId = beaconNodeId;
  g_seq = 0;
  g_decoders.clear ();
}

bool
NrSlBeaconCoverageIsEnabled (void)
{
  return g_enabled;
}

uint32_t
NrSlBeaconCoverageGetBeaconNodeId (void)
{
  return g_beaconNodeId;
}

void
NrSlBeaconCoverageStartNewBeacon (void)
{
  if (!g_enabled)
    {
      return;
    }
  ++g_seq;
  g_decoders.clear ();
}

void
NrSlBeaconCoverageNotifyDecoded (uint32_t rxNodeId)
{
  g_decoders.insert (rxNodeId);
}

NrSlBeaconCoverageSnapshot
NrSlBeaconCoverageGetSnapshot (void)
{
  NrSlBeaconCoverageSnapshot snap;
  snap.seq = g_seq;
  snap.decoded = static_cast<uint32_t> (g_decoders.size ());
  return snap;
}

} // namespace ns3
