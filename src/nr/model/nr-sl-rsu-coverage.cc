/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
#include "nr-sl-rsu-coverage.h"

#include <set>
#include <unordered_map>
#include <unordered_set>

namespace ns3 {

namespace {

bool g_enabled = false;
std::unordered_set<uint32_t> g_rsuNodeIdSet;
std::vector<uint32_t> g_rsuNodeIdOrder; //!< registration order, for deterministic snapshot output

struct PerRsuState
{
  uint32_t txCount {0};
  std::set<uint32_t> decoders;
};
std::unordered_map<uint32_t, PerRsuState> g_state;

} // namespace

void
NrSlRsuCoverageEnable (const std::vector<uint32_t>& rsuNodeIds)
{
  g_enabled = true;
  g_rsuNodeIdSet.clear ();
  g_rsuNodeIdOrder = rsuNodeIds;
  g_state.clear ();
  for (uint32_t id : rsuNodeIds)
    {
      g_rsuNodeIdSet.insert (id);
      g_state[id]; // default-construct entry
    }
}

bool
NrSlRsuCoverageIsEnabled (void)
{
  return g_enabled;
}

bool
NrSlRsuCoverageIsRsuNodeId (uint32_t nodeId)
{
  return g_rsuNodeIdSet.count (nodeId) > 0;
}

void
NrSlRsuCoverageNotifyTx (uint32_t rsuNodeId)
{
  if (!g_enabled)
    {
      return;
    }
  g_state[rsuNodeId].txCount++;
}

void
NrSlRsuCoverageNotifyDecoded (uint32_t rsuNodeId, uint32_t rxNodeId)
{
  if (!g_enabled)
    {
      return;
    }
  g_state[rsuNodeId].decoders.insert (rxNodeId);
}

NrSlRsuCoverageSnapshot
NrSlRsuCoverageGetSnapshotAndReset (void)
{
  NrSlRsuCoverageSnapshot snap;
  for (uint32_t id : g_rsuNodeIdOrder)
    {
      auto it = g_state.find (id);
      if (it == g_state.end ())
        {
          continue;
        }
      NrSlRsuCoverageSnapshot::PerRsu pr;
      pr.rsuNodeId = id;
      pr.txCount = it->second.txCount;
      pr.decoderNodeIds.assign (it->second.decoders.begin (), it->second.decoders.end ());
      snap.perRsu.push_back (pr);

      it->second.txCount = 0;
      it->second.decoders.clear ();
    }
  return snap;
}

} // namespace ns3
