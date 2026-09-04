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

  std::vector<std::string> getVehicleNodeMapIds(); // get all vehicle node ids

  std::map< std::string, std::pair< StationType_t, Ptr<Node> > > get_NodeMap() {return m_NodeMap;};

  void AddStation(std::string id, float x, float y, float z, Ptr<Node> node);

  std::string GetStationId(Ptr<Node> node);

  void SetSionnaUp() {m_sionna = true;};

  // Fired at the end of every SumoSimulationStep tick (after
  // SynchroniseNodeMap/UpdatePositions, so m_NodeMap and positions are
  // current), with the current count of live vehicles (StationType_passengerCar
  // entries in m_NodeMap). Stored as std::function, not a concrete callee
  // type, for the same reason as m_includeNode/m_excludeNode/m_gossipSend
  // above: TraciClient stays unaware of what module the callback belongs to.
  void SetPerTickCallback (std::function<void(uint32_t)> cb);


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
  std::function<void(uint32_t)> m_perTickCallback;

};

} // end namespace ns3

#endif /* TRACI_H */

