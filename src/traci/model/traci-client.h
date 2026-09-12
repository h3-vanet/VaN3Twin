/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

/*
 * Copyright (c) 2018 TU Dresden
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Patrick Schmager <patrick.schmager@tu-dresden.de>
 *          Sebastian Kuehlmorgen <sebastian.kuehlmorgen@tu-dresden.de>
 */

#ifndef TRACI_H
#define TRACI_H

#include <map>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>
#include <string>
#include <functional>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <zmq.h>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"

#include "sumo-TraCIAPI.h"
#include "sumo-TraCIDefs.h"

#include "ns3/socket.h"
#include "ns3/ipv4-address.h"
#include "ns3/vehicle-visualizer.h"

#include "ns3/StationType.h"

#ifdef HAVE_SIONNA
#include "ns3/sionna-connection-handler.h"
#else
#include "ns3/vector.h"
#include <string>
namespace ns3 {
inline void updateLocationInSionna (const std::string &, const Vector &, double, const Vector &) {}
} // namespace ns3
#endif

// No include of v2x-gossip-app.h here — would create a circular link dependency
// (automotive → traci → automotive). The full type is only needed in traci-client.cc.
// RegisterGossipApp accepts Ptr<Application> and casts internally.

#define STARTUP_FCN std::function<Ptr<Node>(std::string,TraciClient::StationTypeTraCI_t)>
#define SHUTDOWN_FCN std::function<void(Ptr<Node>,std::string)>

namespace ns3 {

class TraciClient : public TraCIAPI, public Object
{
public:
  typedef enum {
    StationTypeTraci_vehicle,
    StationTypeTraci_roadSideUnit,
    StationTypeTraci_pedestrian,
    StationTypeTraci_other,
    StationTypeTraci_unspecified
  } StationTypeTraCI_t;

  // register this type with the TypeId system.
  static TypeId GetTypeId (void);

  // constructor and destructor
  TraciClient (void);
  ~TraciClient(void);

  // start up sumo; pass function pointers for including and excluding node functions
  void SumoSetup(STARTUP_FCN includeNode, SHUTDOWN_FCN excludeNode);

  void SumoStop();

  // get associated sumo vehicle for ns3 node
  std::string GetVehicleId(Ptr<Node> node);

  uint32_t GetVehicleMapSize(); // size of vehicle map

  void RegisterGossipApp(const std::string& vehicleId, Ptr<Application> app);
  void RegisterVehicleId(const std::string& sumoId, uint64_t rustId);

  // Register an RSU's raw UDP socket as a gossip-dispatch target under
  // rsuId (e.g. "rsu0"), in the SAME m_gossipSend map the vehicles'
  // GossipApp callbacks live in -- see the RegisterRsuSend definition for
  // why it is shared rather than a parallel map. groupAddr/port must match
  // where the vehicles' V2xGossipApp is bound, or an RSU-originated
  // envelope will reach the sidelink but find no application listening.
  // onSend, if set, is invoked every time this RSU's dispatch lambda
  // actually sends a packet -- e.g. so a caller in a module traci does not
  // depend on (nr) can count transmission attempts without traci itself
  // taking that dependency.
  void RegisterRsuSend(const std::string& rsuId, Ptr<Socket> socket,
                       Ipv4Address groupAddr, uint16_t port,
                       std::function<void()> onSend = nullptr);

  // Register an RSU's RECEIVING application (a V2xGossipApp instance bound
  // to the same group/port vehicles transmit their uplink on) so a
  // broker-bound message it decodes gets forwarded to the broker. This is
  // the symmetric counterpart to RegisterRsuSend: that one lets the broker
  // reach a vehicle over sidelink, this one lets a vehicle's claim/query
  // reach the broker over sidelink -- see OnRsuUplinkReceived. Unlike
  // RegisterRsuSend, this does NOT touch m_gossipSend: an RSU never
  // transmits from this app (its own Send() is never called), it only
  // listens.
  void RegisterRsuReceive(const std::string& rsuId, Ptr<Application> app);

  // Turn on the rsu_delivery.csv instrumentation (see OUTPUT in the
  // scenario file / nr-sl-rsu-coverage.h for the analogous rsu_coverage
  // mechanism). Disabled by default: with this never called (i.e. every
  // --rsu-count=0 run), every delivery-log code path below is a no-op
  // gated on m_rsuDeliveryEnabled, so the leaderless arm is byte-for-byte
  // unaffected and no file is produced.
  void EnableRsuDeliveryLog(const std::string& path);

  // SCI Format-2's source id is 8 bits, so at most 255 distinct sidelink
  // transmitters can coexist without two of them aliasing mod 256 (see
  // nr-sl-sci-f2-header.cc). This is NOT a startup/pool-size check: IMSIs
  // (and therefore L2/source ids) are assigned to every pool node at
  // construction regardless of whether that node ever actually transmits,
  // but the observed SIGSEGV requires two ALIASING nodes to both actually
  // be transmitting/decoding at the same time -- a preallocated pool can
  // be larger than 255 nodes and still run to completion if a given run
  // never actually exercises more than 255 of them, and conversely a
  // run's total pool size alone says nothing about when (or whether) that
  // threshold is crossed during THIS run. So this is called at the one
  // place that already knows a node has just gone from "silent" to
  // "actually transmitting on the sidelink" -- see the call site in
  // ProcessGossipIn (covers every vehicle send, RSU downlink relay, and
  // RSU uplink forward, since they all share that one dispatch call) and
  // the coverage-probe beacon's own TX site in the scenario file. A
  // caller like the beacon that transmits from a Node not otherwise
  // known to TraciClient is expected to call this itself once it starts.
  // Idempotent per node id; the abort only ever fires the first time the
  // 256th DISTINCT node is seen actually transmitting, not on every call.
  void NotifySidelinkTransmit(uint32_t nodeId);

  std::vector<std::string> getVehicleNodeMapIds(); // get all vehicle node ids

  std::map< std::string, std::pair< StationType_t, Ptr<Node> > > get_NodeMap() {return m_NodeMap;};

  void AddStation(std::string id, float x, float y, float z, Ptr<Node> node);

  std::string GetStationId(Ptr<Node> node);

  void SetSionnaUp() {m_sionna = true;};

  // Fired at the end of every SumoSimulationStep tick (after
  // SynchroniseNodeMap/UpdatePositions, so m_NodeMap and positions are
  // current), with the ns-3 Node IDs of the current live vehicles
  // (StationType_passengerCar entries in m_NodeMap) — the full id set, not
  // just a count, so a caller can intersect it against some other node-id
  // set (e.g. distinct decoders of a broadcast) rather than only counting
  // it. Stored as std::function, not a concrete callee type, for the same
  // reason as m_includeNode/m_excludeNode/m_gossipSend above: TraciClient
  // stays unaware of what module the callback belongs to.
  void SetPerTickCallback (std::function<void(const std::vector<uint32_t>&)> cb);


private:
  // perform sumo simulation for a certain time step
  void SumoSimulationStep(void);

  // get current positions from sumo vehicles and update corresponding ns3 nodes positions
  void UpdatePositions(void);

  // get new (departed) and removed (arrived) vehicles from sumo
  void GetSumoVehicles(std::vector<std::string>& sumoVehicles);

  // synchronise ns3 nodes with sumo vehicles
  void SynchroniseNodeMap(void);

  // build command line string for sumo start up
  std::string GetSumoCmdString (void);

  // map every sumo vehicle/pedestrian to a ns3 node
  std::map< std::string, std::pair< StationType_t, Ptr<Node> > > m_NodeMap;

  // a vehicle is untracked if it is simulated in sumo but not linked to a ns3 node because of an penetration rate < 1.0
  std::vector<std::string> m_untrackedVehicles;

  // function pointers to node include/exclude functions 
  STARTUP_FCN m_includeNode;
  SHUTDOWN_FCN m_excludeNode;

  // port handling functionality for multiple parallel simulations
  static bool PortFreeCheck (uint32_t portNum);
  static uint32_t GetFreePort (uint32_t portNum=10000);

  // simulation specific data members
  std::string m_sumoAddCmdOpt;
  std::string m_sumoCommand;
  std::string m_sumoConfigPath;
  std::string m_sumoBinaryPath;
  uint16_t m_sumoPort;
  bool m_sumoGUI;

  double m_penetrationRate;
  ns3::Time m_synchInterval;
  ns3::Time m_startTime;
  
  bool m_sumoLogFile;
  bool m_sumoStepLog;
  double m_altitude;
  int m_sumoSeed;
  ns3::Time m_sumoWaitForSocket;

  // Flag pedestrians list empty
  bool m_pedlist_empty = true;

  Ptr<vehicleVisualizer> m_vehicle_visualizer;
  std::string m_netns_name;
  void terminateVehicleVisualizer (void);

  bool m_sionna = false;

  // ZMQ PUSH socket — vehicle events (env ZMQ_PUB_PORT, default 5555, + ZMQ_PORT_OFFSET)
  void*  m_zmq_context = nullptr;
  void*  m_zmq_pub     = nullptr;
  void   zmqPublish (const char* json);
  void   zmqPublishCritical (const char* json);

  // ZMQ PULL socket — vehicle commands from bridge (env ZMQ_CMD_PORT, default 5558, + ZMQ_PORT_OFFSET)
  void*  m_zmq_cmd     = nullptr;
  void   ProcessCommands ();

  // Gossip relay — PULL (env ZMQ_GOSSIP_IN_PORT, default 5560) / PUSH (env ZMQ_GOSSIP_OUT_PORT, default 5561) + ZMQ_PORT_OFFSET
  void*  m_zmq_gossip_in  = nullptr;
  void*  m_zmq_gossip_out = nullptr;
  // Store Send callbacks as std::function to avoid pulling V2xGossipApp type into this header
  std::unordered_map<std::string, std::function<void(const uint8_t*, uint32_t)>> m_gossipSend;
  std::unordered_map<std::string, uint64_t>           m_sumo_to_u64;

  // Which m_gossipSend keys are RSUs, not vehicles -- see RegisterRsuSend.
  // Used only to keep aggregate, scalar stats (e.g. the "vehicles with
  // gossip app" density proxy) from counting RSUs as vehicles; per-key
  // counters/logs are left keyed by sumo_id as before, already
  // distinguishable by the "rsu" prefix.
  std::unordered_set<std::string>                     m_rsuSumoIds;

  // Per-vehicle gossip metrics for visualizer (color coding, delivery ratio, neighbor count)
  std::unordered_map<std::string, uint32_t>              m_gossipTxCount;
  std::unordered_map<std::string, uint32_t>              m_gossipRxCount;
  std::unordered_map<std::string, std::set<std::string>> m_gossipNeighbors;
  std::unordered_map<std::string, uint32_t>              m_lastGossipTxSent;
  std::unordered_map<std::string, uint32_t>              m_lastGossipRxSent;

  // Per-vehicle gossip summary logging (one log line per 100 msgs OR 30s sim-time per vehicle)
  std::unordered_map<std::string, uint32_t> m_gossipTxLog;
  std::unordered_map<std::string, uint32_t> m_gossipRxLog;
  std::unordered_map<std::string, uint32_t> m_gossipTxTotal;
  std::unordered_map<std::string, uint32_t> m_gossipRxTotal;
  std::unordered_map<std::string, double>   m_gossipLastLogTime;

  // Async gossip logger — background thread writes to /tmp/gossip_ns3.log
  std::queue<std::string>  m_logQueue;
  std::mutex               m_logMutex;
  std::condition_variable  m_logCv;
  std::thread              m_logThread;
  bool                     m_logRunning = false;
  void   GossipLog(const std::string& msg);
  void   LogThreadFn();

  // Vehicles requested to be removed via RemoveVehicle command — drained by SynchroniseNodeMap
  std::unordered_set<std::string> m_pendingRemoval;

  // Visualizer rate limit
  double m_lastVisUpdateTime = -1.0;

  void   ProcessGossipIn();
  void   OnGossipReceived(const std::string& receiverSumoId,
                          const uint8_t* data, uint32_t len);

  // See SetPerTickCallback().
  std::function<void(const std::vector<uint32_t>&)> m_perTickCallback;

  // Uplink (vehicle -> broker) sidelink path -- see RegisterRsuReceive.
  // Handles a broker-bound envelope ("dst":"broker") a specific RSU has
  // decoded: forwards it over the (ideal, wired -- see SumoSetup) RSU ->
  // broker backhaul and updates the delivery log below.
  void   OnRsuUplinkReceived(const std::string& rsuId,
                             const uint8_t* data, uint32_t len);

  // RSU -> broker backhaul. This link is NOT modeled over the radio: an
  // RSU's own connection back to the broker is assumed wired and ideal
  // (the standard assumption for RSU backhaul), same as how the broker's
  // ZMQ link to ns-3 itself is not part of the simulated radio. Only the
  // vehicle <-> RSU hop over NR-V2X sidelink is what this change makes
  // symmetric. PUSH (env ZMQ_UPLINK_OUT_PORT, default 5562) + ZMQ_PORT_OFFSET.
  void*  m_zmq_uplink_out = nullptr;

  // --- rsu_delivery.csv instrumentation (see EnableRsuDeliveryLog) ---
  //
  // Separate from nr-sl-rsu-coverage.h's coverage probe (which counts any
  // live decoder of any RSU transmission, with no notion of an intended
  // recipient): this tags each broker-bound/broker-originated envelope
  // with the "msg_id" the wire envelope already carries (see
  // ProcessGossipIn/OnRsuUplinkReceived) and matches TX against RX by that
  // id, at the same application-layer reassembly point OnGossipReceived
  // and the RSU's receive app already use for ordinary gossip -- i.e.
  // "delivered" means the full envelope came back up through the UDP
  // socket at the intended node, not "some decode happened in the same
  // window". A msg_id of 0 is treated as absent (the request-id generator
  // on the Rust side is assumed to never issue 0).
  bool        m_rsuDeliveryEnabled = false;
  std::string m_rsuDeliveryLogPath;

  struct RsuDeliveryRow
  {
    char        direction {'d'};   // 'd' = down (broker->vehicle), 'u' = up (vehicle->broker)
    uint64_t    msgId {0};
    double      tTx {0.0};
    std::string srcNode;           // down: "broker"; up: sending vehicle's ns-3 Node ID
    std::string dst;               // down: recipient vehicle's ns-3 Node ID; up: "broker"
    std::string servingRsu;        // down only -- the RSU selected to transmit
    std::set<std::string> decodingRsus; // up only -- every distinct RSU that decoded it
    uint32_t    nDecoders {0};     // up: decodingRsus.size(); down: 1 once delivered, else 0
    double      distM {-1.0};      // vehicle-to-nearest-RSU distance at t_tx; -1 = no RSU existed
    bool        vehicleLiveAtTx {false};
    bool        delivered {false};
    double      tRx {-1.0};
    double      latencyMs {-1.0};
  };
  // See NotifySidelinkTransmit(): distinct ns-3 Node IDs seen actually
  // transmitting on the sidelink so far this run.
  std::unordered_set<uint32_t> m_sidelinkTransmitterIds;

  std::vector<RsuDeliveryRow> m_rsuDeliveryRows;
  // Index into m_rsuDeliveryRows, keyed by "d:<msg_id>" / "u:<msg_id>" so
  // the two directions' id spaces can never collide even if the Rust side
  // does not itself partition them.
  std::unordered_map<std::string, size_t> m_rsuDeliveryIndex;

  // Distance/id of the RSU nearest to a given position, scanning
  // m_NodeMap the same way the existing broker-envelope dispatch above
  // does -- factored out so the uplink delivery-log TX site (a vehicle's
  // own position) and the downlink one (a destination vehicle's position)
  // share one implementation instead of two copies of the same loop.
  struct NearestRsu { bool have {false}; double dist {0.0}; std::string id; };
  NearestRsu FindNearestRsu (const Vector& pos);

  void WriteRsuDeliveryLog();

};

} // end namespace ns3

#endif /* TRACI_H */

