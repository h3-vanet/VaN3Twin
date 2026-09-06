/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

//#include "ns3/automotive-module.h"
#include "ns3/emergencyVehicleAlert-helper.h"
#include "ns3/emergencyVehicleAlert.h"
#include "ns3/v2x-gossip-app-helper.h"
#include "ns3/v2x-gossip-app.h"
#include "ns3/traci-module.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/nr-sl-beacon-coverage.h"
#include "ns3/lte-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/log.h"
#include "ns3/antenna-module.h"
#include <iomanip>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <vector>
#include <unistd.h>
#include "ns3/sumo_xml_parser.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <unistd.h>
#include "ns3/core-module.h"
#include "ns3/tee-streambuf.h"


using namespace ns3;

// Saved original streambufs for teed stdout/stderr
static std::streambuf *g_old_cout_buf = nullptr;
static std::streambuf *g_old_cerr_buf = nullptr;
static TeeStreamBuf *g_tee_cout = nullptr;
static TeeStreamBuf *g_tee_cerr = nullptr;

NS_LOG_COMPONENT_DEFINE("v2v-nrv2x");

/**
 * \brief Get sidelink bitmap from string
 * \param slBitMapString The sidelink bitmap string
 * \param slBitMapVector The vector passed to store the converted sidelink bitmap
 */
void
GetSlBitmapFromString (std::string slBitMapString, std::vector <std::bitset<1> > &slBitMapVector)
{
  static std::unordered_map<std::string, uint8_t> lookupTable =
  {
    { "0", 0 },
    { "1", 1 },
  };

  std::stringstream ss (slBitMapString);
  std::string token;
  std::vector<std::string> extracted;

  while (std::getline (ss, token, '|'))
    {
      extracted.push_back (token);
    }

  for (const auto & v : extracted)
    {
      if (lookupTable.find (v) == lookupTable.end ())
        {
          NS_FATAL_ERROR ("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
      slBitMapVector.push_back (lookupTable[v] & 0x01);
    }
}

/**
 * \brief Coverage-probe beacon: send a small fixed-size datagram over the
 *        sidelink groupcast bearer, then reschedule itself.
 *
 * Off by default (only ever scheduled when --beacon is passed). Mirrors
 * the self-reschedule pattern used by [nr-plr]'s periodic reporter in
 * nr-spectrum-phy.cc, at 1 Hz instead of 1/10 Hz.
 *
 * Calls NrSlBeaconCoverageStartNewBeacon() exactly once per invocation,
 * i.e. once per logical beacon — deliberately NOT from the PHY TX path
 * (NrSpectrumPhy::StartTxSlDataFrames), which fires once per MAC-layer
 * transmission *attempt* and would fire multiple times per logical
 * beacon under EnableBlindReTx (see nr-sl-beacon-coverage.h).
 */
void
ScheduleSlBeaconTx (Ptr<Socket> socket, Ipv4Address groupAddr, uint16_t port, uint32_t payloadBytes)
{
  NrSlBeaconCoverageStartNewBeacon ();
  Ptr<Packet> pkt = Create<Packet> (payloadBytes);
  socket->SendTo (pkt, 0, InetSocketAddress (groupAddr, port));
  Simulator::Schedule (Seconds (1.0), &ScheduleSlBeaconTx, socket, groupAddr, port, payloadBytes);
}


int
main (int argc, char *argv[])
{

  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";

  /*** 0.a App Options ***/
  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = false;
  double sumo_updates = 0.01;
  std::string csv_name;
  std::string csv_name_cumulative;
  std::string sumo_netstate_file_name;
  std::string log_prefix = "/sim/logs/van3twin";
  bool vehicle_vis = false;
  double sumo_wait_for_socket = 5.0; // seconds to wait for SUMO to open TraCI socket

  int numberOfNodes;
  uint32_t nodeCounter = 0;

  double penetrationRate = 0.7;

  // Coverage-probe beacon (off by default; see TASK 1/2/3/4 of the
  // instrumentation-survey follow-up). --beacon creates a static
  // non-vehicle sidelink node at the map center and transmits a 1 Hz
  // probe from it. --beacon-node-id independently selects which node id
  // the decode-coverage counter/emitter (Tasks 3-4) tracks, so the same
  // counting code works unchanged once a real broker node exists
  // elsewhere: if --beacon is also given and --beacon-node-id is not
  // explicitly set, it defaults to the probe node's own id.
  bool enableBeacon = false;
  int64_t beaconNodeId = -1;

  // Generic RSU nodes (--rsu-count, off by default: 0 nodes), for a later
  // centralized-broker arm. DECISION: kept as a mechanism separate from
  // --beacon above, not "beacon is rsu-count==1" — they are conceptually
  // distinct:
  //   - --beacon is specifically the coverage-probe transmitter: it drives
  //     ScheduleSlBeaconTx() (1 Hz TX) and is the node NrSlBeaconCoverage
  //     tracks via --beacon-node-id. That semantics doesn't belong on a
  //     generic RSU.
  //   - --rsu-count nodes carry no beacon-transmission or coverage-counter
  //     semantics of their own; they are just addressable NR-V2X sidelink
  //     endpoints for a later task (the Zenoh bridge) to drive individually.
  //   - Keeping --beacon's existing code path completely untouched removes
  //     any risk of changing its behavior (including RNG-stream numbering)
  //     for the already-completed experimental runs that used it, which
  //     unifying the two mechanisms behind one code path would risk.
  // Both still reuse the same already-configured nrHelper/nrSlHelper/
  // allBwps/bwpIdContainer/slPreConfigNr/internet/ipv4RoutingHelper, the
  // same way --beacon does -- "same stack", not "same node".
  uint32_t rsuCount = 0;

  xmlDocPtr rou_xml_file;
  double m_baseline_prr = 150.0;
  bool m_metric_sup = false;


  // Simulation parameters.
  double simTime = 100.0;
  //Sidelink bearers activation time
  Time slBearersActivationTime = Seconds (2.0);


  // NR parameters. We will take the input from the command line, and then we
  // will pass them inside the NR module.
  double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
  uint16_t bandwidthBandSl = 400;
  double txPower = 23; //dBm
  std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
  std::string slBitMap = "1|1|1|1|1|1|1|1|1|1";
  uint16_t numerologyBwpSl = 2;
  uint16_t slSensingWindow = 100; // T0 in ms
  uint16_t slSelectionWindow = 5; // T2min
  uint16_t slSubchannelSize = 10;
  uint16_t slMaxNumPerReserve = 3;
  double slProbResourceKeep = 0.0;
  uint16_t slMaxTxTransNumPssch = 5;
  uint16_t reservationPeriod = 20; // in ms
  bool enableSensing = false;
  uint16_t t1 = 2;
  uint16_t t2 = 81;
  int slThresPsschRsrp = -128;
  bool enableChannelRandomness = false;
  uint16_t channelUpdatePeriod = 500; //ms
  uint8_t mcs = 14;

  /*
   * From here, we instruct the ns3::CommandLine class of all the input parameters
   * that we may accept as input, as well as their description, and the storage
   * variable.
   */
  CommandLine cmd;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("sumo-wait", "Seconds to wait for SUMO to open TraCI socket", sumo_wait_for_socket);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);
  cmd.AddValue ("penetrationRate", "Rate of vehicles equipped with wireless communication devices", penetrationRate);
  cmd.AddValue ("log-prefix", "Prefix for the log file name (e.g. 'bridge' -> bridge.log.YYYY-MM-DD)", log_prefix);

  cmd.AddValue ("simTime",
                "Simulation time in seconds",
                simTime);
  cmd.AddValue ("slBearerActivationTime",
                "Sidelik bearer activation time in seconds",
                slBearersActivationTime);
  cmd.AddValue ("centralFrequencyBandSl",
                "The central frequency to be used for Sidelink band/channel",
                centralFrequencyBandSl);
  cmd.AddValue ("bandwidthBandSl",
                "The system bandwidth to be used for Sidelink",
                bandwidthBandSl);
  cmd.AddValue ("txPower",
                "total tx power in dBm",
                txPower);
  cmd.AddValue ("tddPattern",
                "The TDD pattern string",
                tddPattern);
  cmd.AddValue ("slBitMap",
                "The Sidelink bitmap string",
                slBitMap);
  cmd.AddValue ("numerologyBwpSl",
                "The numerology to be used in Sidelink bandwidth part",
                numerologyBwpSl);
  cmd.AddValue ("slSensingWindow",
                "The Sidelink sensing window length in ms",
                slSensingWindow);
  cmd.AddValue ("slSelectionWindow",
                "The parameter which decides the minimum Sidelink selection "
                "window length in physical slots. T2min = slSelectionWindow * 2^numerology",
                slSelectionWindow);
  cmd.AddValue ("slSubchannelSize",
                "The Sidelink subchannel size in RBs",
                slSubchannelSize);
  cmd.AddValue ("slMaxNumPerReserve",
                "The parameter which indicates the maximum number of reserved "
                "PSCCH/PSSCH resources that can be indicated by an SCI.",
                slMaxNumPerReserve);
  cmd.AddValue ("slProbResourceKeep",
                "The parameter which indicates the probability with which the "
                "UE keeps the current resource when the resource reselection"
                "counter reaches zero.",
                slProbResourceKeep);
  cmd.AddValue ("slMaxTxTransNumPssch",
                "The parameter which indicates the maximum transmission number "
                "(including new transmission and retransmission) for PSSCH.",
                slMaxTxTransNumPssch);
  cmd.AddValue ("ReservationPeriod",
                "The resource reservation period in ms",
                reservationPeriod);
  cmd.AddValue ("enableSensing",
                "If true, it enables the sensing based resource selection for "
                "SL, otherwise, no sensing is applied",
                enableSensing);
  cmd.AddValue ("t1",
                "The start of the selection window in physical slots, "
                "accounting for physical layer processing delay",
                t1);
  cmd.AddValue ("t2",
                "The end of the selection window in physical slots",
                t2);
  cmd.AddValue ("slThresPsschRsrp",
                "A threshold in dBm used for sensing based UE autonomous resource selection",
                slThresPsschRsrp);
  cmd.AddValue ("enableChannelRandomness",
                "Enable shadowing and channel updates",
                enableChannelRandomness);
  cmd.AddValue ("channelUpdatePeriod",
                "The channel update period in ms",
                channelUpdatePeriod);
  cmd.AddValue ("mcs",
                "The MCS to used for sidelink",
                mcs);
  cmd.AddValue ("beacon",
                "Create a static non-vehicle sidelink node at the map "
                "center and transmit a 1 Hz coverage-probe beacon from "
                "it (off by default)",
                enableBeacon);
  cmd.AddValue ("beacon-node-id",
                "ns-3 Node ID whose sidelink broadcasts the decode-coverage "
                "counter should track (off/disabled unless set). If "
                "--beacon is also given and this is left unset, it "
                "defaults to the probe node's own id",
                beaconNodeId);
  cmd.AddValue ("rsu-count",
                "Number of generic non-vehicle sidelink RSU nodes to create "
                "(off by default: 0). Distributed over the SUMO map's "
                "bounding box on a uniform grid. Separate mechanism from "
                "--beacon -- see the rsuCount declaration for why.",
                rsuCount);


  // Parse the command line
  cmd.Parse (argc, argv);

  // Set up tee: duplicate all stdout/stderr to a log file
  {
    std::string log_file = defaultLogFileName (log_prefix);
    g_old_cout_buf = std::cout.rdbuf ();
    g_old_cerr_buf = std::cerr.rdbuf ();
    g_tee_cout = new TeeStreamBuf (g_old_cout_buf, log_file);
    g_tee_cerr = new TeeStreamBuf (g_old_cerr_buf, log_file);
    std::cout.rdbuf (g_tee_cout);
    std::cerr.rdbuf (g_tee_cerr);
    std::cout << "[log] tee started, writing to " << log_file << std::endl;
  }

  if (verbose)
    {
      LogComponentEnable ("v2v-nrv2x", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

  /*
   * Check if the frequency is in the allowed range.
   * If you need to add other checks, here is the best position to put them.
   */
  NS_ABORT_IF (centralFrequencyBandSl > 6e9);

  /*
   * Default values for the simulation.
   */
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));


  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /***  Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");
  /*
   * Create a NodeContainer for all the UEs
   */
  NodeContainer allSlUesContainer;
  allSlUesContainer.Create(numberOfNodes);

  /*
   * Assign mobility to the UEs.
   */
  MobilityHelper mobility;
  mobility.Install (allSlUesContainer);

  /*
   * Setup the NR module. We create the various helpers needed for the
   * NR simulation:
   * - EpcHelper, which will setup the core network
   * - NrHelper, which takes care of creating and connecting the various
   * part of the NR stack
   */
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  // Put the pointers inside nrHelper
  nrHelper->SetEpcHelper (epcHelper);

  /*
   * Spectrum division. We create one operational band, containing
   * one component carrier, and a single bandwidth part
   * centered at the frequency specified by the input parameters.
   * We will use the StreetCanyon channel modeling.
   */
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;

  /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf
   * creates a single BWP per CC
   */
  CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::V2V_Highway);
  //CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::CV2X_UrbanMicrocell);

  // By using the configuration created, it is time to make the operation bands
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc (bandConfSl);

  /*
   * The configured spectrum division is:
   * ------------Band1--------------
   * ------------CC1----------------
   * ------------BwpSl--------------
   */
  if (enableChannelRandomness)
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true));
    }
  else
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
    }

  /*
   * Initialize channel and pathloss, plus other things inside bandSl. If needed,
   * the band configuration can be done manually, but we leave it for more
   * sophisticated examples. For the moment, this method will take care
   * of all the spectrum initialization needs.
   */
  nrHelper->InitializeOperationBand (&bandSl);
  allBwps = CcBwpCreator::GetAllBwps ({bandSl});


  /*
   * Now, we can setup the attributes. We can have three kind of attributes:
   */

  /*
   * Antennas for all the UEs
   * We are not using beamforming in SL, rather we are using
   * quasi-omnidirectional transmission and reception, which is the default
   * configuration of the beams.
   *
   * Following attribute would be common for all the UEs
   */
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));  //following parameter has no impact at the moment because:
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));

  nrHelper->SetUeMacAttribute ("EnableSensing", BooleanValue (enableSensing));
  nrHelper->SetUeMacAttribute ("T1", UintegerValue (static_cast<uint8_t> (t1)));
  nrHelper->SetUeMacAttribute ("T2", UintegerValue (t2));
  nrHelper->SetUeMacAttribute ("ActivePoolId", UintegerValue (0));
  nrHelper->SetUeMacAttribute ("ReservationPeriod", TimeValue (MilliSeconds (reservationPeriod)));
  nrHelper->SetUeMacAttribute ("NumSidelinkProcess", UintegerValue (4));
  nrHelper->SetUeMacAttribute ("EnableBlindReTx", BooleanValue (true));
  nrHelper->SetUeMacAttribute ("SlThresPsschRsrp", IntegerValue (slThresPsschRsrp));

  uint8_t bwpIdForGbrMcptt = 0;

  nrHelper->SetBwpManagerTypeId (TypeId::LookupByName ("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_MC_PUSH_TO_TALK", UintegerValue (bwpIdForGbrMcptt));

  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert (bwpIdForGbrMcptt);

  /*
   * We have configured the attributes we needed. Now, install and get the pointers
   * to the NetDevices, which contains all the NR stack:
   */
  NetDeviceContainer allSlUesNetDeviceContainer = nrHelper->InstallUeDevice (allSlUesContainer, allBwps);

  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = allSlUesNetDeviceContainer.Begin (); it != allSlUesNetDeviceContainer.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  /*
   * Configure Sidelink. We create the following helpers needed for the
   * NR Sidelink, i.e., V2X simulation:
   * - NrSlHelper, which will configure the UEs protocol stack to be ready to
   *   perform Sidelink related procedures.
   * - EpcHelper, which takes care of triggering the call to EpcUeNas class
   *   to establish the NR Sidelink bearer(s). We note that, at this stage
   *   just communicate the pointer of already instantiated EpcHelper object,
   *   which is the same pointer communicated to the NrHelper above.
   */
  Ptr<NrSlHelper> nrSlHelper = CreateObject <NrSlHelper> ();
  // Put the pointers inside NrSlHelper
  nrSlHelper->SetEpcHelper (epcHelper);

  /*
   * Set the SL error model and AMC
   * Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1,
   *                   ns3::NrEesmIrT2, ns3::NrLteMiErrorModel
   * AMC type: NrAmc::ShannonModel or NrAmc::ErrorModel
   */
  std::string errorModel = "ns3::NrLteMiErrorModel";
  nrSlHelper->SetSlErrorModel (errorModel);
  nrSlHelper->SetUeSlAmcAttribute ("AmcModel", EnumValue (NrAmc::ErrorModel));

  /*
   * Set the SL scheduler attributes
   * In this example we use NrSlUeMacSchedulerSimple scheduler, which uses
   * fix MCS value
   */
  nrSlHelper->SetNrSlSchedulerTypeId (NrSlUeMacSchedulerSimple::GetTypeId());
  nrSlHelper->SetUeSlSchedulerAttribute ("FixNrSlMcs", BooleanValue (true));
  nrSlHelper->SetUeSlSchedulerAttribute ("InitialNrSlMcs", UintegerValue (mcs));

  /*
   * Very important method to configure UE protocol stack, i.e., it would
   * configure all the SAPs among the layers, setup callbacks, configure
   * error model, configure AMC, and configure ChunkProcessor in Interference
   * API.
   */
  nrSlHelper->PrepareUeForSidelink (allSlUesNetDeviceContainer, bwpIdContainer);


  /*
   * Start preparing for all the sub Structs/RRC Information Element (IEs)
   * of LteRrcSap::SidelinkPreconfigNr. This is the main structure, which would
   * hold all the pre-configuration related to Sidelink.
   */

  //SlResourcePoolNr IE
  LteRrcSap::SlResourcePoolNr slResourcePoolNr;
  //get it from pool factory
  Ptr<NrSlCommPreconfigResourcePoolFactory> ptrFactory = Create<NrSlCommPreconfigResourcePoolFactory> ();
  /*
   * Above pool factory is created to help the users of the simulator to create
   * a pool with valid default configuration. Please have a look at the
   * constructor of NrSlCommPreconfigResourcePoolFactory class.
   *
   * In the following, we show how one could change those default pool parameter
   * values as per the need.
   */
  std::vector <std::bitset<1> > slBitMapVector;
  GetSlBitmapFromString (slBitMap, slBitMapVector);
  NS_ABORT_MSG_IF (slBitMapVector.empty (), "GetSlBitmapFromString failed to generate SL bitmap");
  ptrFactory->SetSlTimeResources (slBitMapVector);
  ptrFactory->SetSlSensingWindow (slSensingWindow); // T0 in ms
  ptrFactory->SetSlSelectionWindow (slSelectionWindow);
  ptrFactory->SetSlFreqResourcePscch (10); // PSCCH RBs
  ptrFactory->SetSlSubchannelSize (slSubchannelSize);
  ptrFactory->SetSlMaxNumPerReserve (slMaxNumPerReserve);
  //Once parameters are configured, we can create the pool
  LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool ();
  slResourcePoolNr = pool;

  //Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
  LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
  slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
  //Pool id, ranges from 0 to 15
  uint16_t poolId = 0;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
  slResourcePoolIdNr.id = poolId;
  slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
  slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

  //Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
  //Array for pools, we insert the pool in the array as per its poolId
  slBwpPoolConfigCommonNr.slTxPoolSelectedNormal [slResourcePoolIdNr.id] = slresoPoolConfigNr;

  //Configure the BWP IE
  LteRrcSap::Bwp bwp;
  bwp.numerology = numerologyBwpSl;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = bandwidthBandSl;

  //Configure the SlBwpGeneric IE
  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum (14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum (0);

  //Configure the SlBwpConfigCommonNr IE
  LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
  slBwpConfigCommonNr.haveSlBwpGeneric = true;
  slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
  slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

  //Configure the SlFreqConfigCommonNr IE, which hold the array to store
  //the configuration of all Sidelink BWP (s).
  LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
  //Array for BWPs. Here we will iterate over the BWPs, which
  //we want to use for SL.
  for (const auto &it:bwpIdContainer)
    {
      // it is the BWP id
      slFreConfigCommonNr.slBwpList [it] = slBwpConfigCommonNr;
    }

  //Configure the TddUlDlConfigCommon IE
  LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
  tddUlDlConfigCommon.tddPattern = tddPattern;

  //Configure the SlPreconfigGeneralNr IE
  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
  slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

  //Configure the SlUeSelectedConfig IE
  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  NS_ABORT_MSG_UNLESS (slProbResourceKeep <= 1.0, "slProbResourceKeep value must be between 0 and 1");
  slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
  //Configure the SlPsschTxParameters IE
  LteRrcSap::SlPsschTxParameters psschParams;
  psschParams.slMaxTxTransNumPssch = static_cast<uint8_t> (slMaxTxTransNumPssch);
  //Configure the SlPsschTxConfigList IE
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
  pscchTxConfigList.slPsschTxParameters [0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

  /*
   * Finally, configure the SidelinkPreconfigNr. This is the main structure
   * that needs to be communicated to NrSlUeRrc class
   */
  LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
  slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
  slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfigNr.slPreconfigFreqInfoList [0] = slFreConfigCommonNr;

  //Communicate the above pre-configuration to the NrSlHelper
  nrSlHelper->InstallNrSlPreConfiguration (allSlUesNetDeviceContainer, slPreConfigNr);

  /****************************** End SL Configuration ***********************/

  /*
   * Fix the random streams
   */
  int64_t stream = 1;
  stream += nrHelper->AssignStreams (allSlUesNetDeviceContainer, stream);
  stream += nrSlHelper->AssignStreams (allSlUesNetDeviceContainer, stream);

  // Coverage-probe beacon node containers/socket (TASK 1/2, off by
  // default): declared here, constructed later — see the consolidated,
  // strictly-after-all-vehicle-construction block below, right after
  // vehicle sidelink bearer activation.
  NodeContainer beaconNodeContainer;
  NetDeviceContainer beaconNetDeviceContainer;
  Ptr<Node> beaconNode;
  Ptr<Socket> beaconSocket;

  // Generic RSU nodes (TASK "interface shape", off by default: rsuCount==0):
  // constructed later in their own block, separate from the beacon's above.
  // rsuNodeContainer/rsuNetDeviceContainer/rsuSockets are the addressable
  // interface a later task uses to drive a specific RSU: node i's ns-3 Node
  // is rsuNodeContainer.Get(i), its NR-V2X NetDevice is
  // rsuNetDeviceContainer.Get(i), and its already-bound UDP socket (same
  // group/port as the vehicles and the beacon) is rsuSockets[i] -- e.g.
  // rsuSockets[i]->SendTo(pkt, 0, InetSocketAddress(groupAddress4, port))
  // transmits from RSU i specifically. All three are sized rsuCount and
  // indexed identically, once the construction block below has run.
  NodeContainer rsuNodeContainer;
  NetDeviceContainer rsuNetDeviceContainer;
  std::vector<Ptr<Socket>> rsuSockets;

  /*
   * if enableOneTxPerLane is true:
   *
   * Divide the UEs in transmitting UEs and receiving UEs. Each lane can
   * have only odd number of UEs, and on each lane middle UE would
   * be the transmitter.
   *
   * else:
   *
   * All the UEs can transmit and receive
   */
  NodeContainer txSlUes;
  NodeContainer rxSlUes;
  NetDeviceContainer txSlUesNetDevice;
  NetDeviceContainer rxSlUesNetDevice;
  txSlUes.Add (allSlUesContainer);
  rxSlUes.Add (allSlUesContainer);
  txSlUesNetDevice.Add (allSlUesNetDeviceContainer);
  rxSlUesNetDevice.Add (allSlUesNetDeviceContainer);

  /*
   * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
   * configured time.
   *
   * This example supports IPV4 and IPV6
   */

  InternetStackHelper internet;
  internet.Install (allSlUesContainer);
  uint32_t dstL2Id = 255;
  Ipv4Address groupAddress4 ("225.0.0.0");     //use multicast address as destination

  Address remoteAddress;
  Address localAddress;
  uint16_t port = 8000;
  Ptr<LteSlTft> tft;

  // Matches V2xGossipApp's own "Port" attribute default (see
  // v2x-gossip-app.cc), named explicitly here -- rather than left as an
  // implicit default -- so RSU gossip dispatch (below) targets the exact
  // port vehicles' GossipApp is bound to instead of relying on an
  // un-named default staying in sync by accident. "port" above (8000) is
  // a different, disjoint port used only by the coverage-probe beacon;
  // it is not where any application on a vehicle node listens.
  uint16_t gossipPort = 8001;

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (allSlUesNetDeviceContainer);

  // set the default gateway for the UE
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < allSlUesContainer.GetN (); ++u)
    {
      Ptr<Node> ueNode = allSlUesContainer.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  remoteAddress = InetSocketAddress (groupAddress4, port);
  localAddress = InetSocketAddress (Ipv4Address::GetAny (), port);

  tft = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  tft = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  /*
   * Coverage-probe beacon node (TASK 1/2, off by default): a single
   * non-vehicle node, created independently of allSlUesContainer so it
   * never consumes a nodeCounter/vehicle-pool index (see the
   * setupNewWifiNode lambda below), given the same NR-V2X sidelink stack
   * and TX power as the vehicles by reusing the same already-configured
   * nrHelper/nrSlHelper/allBwps/bwpIdContainer/slPreConfigNr/internet/
   * ipv4RoutingHelper. Its real position (map center) is set later, once
   * sumoClient exists — see "6. Setup Traci and start SUMO" below.
   *
   * Placed here, strictly after every vehicle-related object-construction
   * statement above (NR device install, AssignStreams, internet stack,
   * bearer activation) — not interleaved with any of it — so that
   * creating the beacon can only ever consume RNG stream numbering after
   * every vehicle-related consumer, never in between. ns-3 assigns
   * automatic/implicit stream indices to RandomVariableStream objects in
   * construction order; only the indices AssignStreams() explicitly
   * re-pins are provably order-independent, and this scenario has no
   * explicit AssignStreams() call for the internet stack. Keeping this
   * block last, all in one place, removes that risk by construction
   * instead of relying on an audit of ns-3/internet-module internals.
   */
  if (enableBeacon)
    {
      beaconNodeContainer.Create (1);
      beaconNode = beaconNodeContainer.Get (0);
      MobilityHelper beaconMobility;
      beaconMobility.Install (beaconNodeContainer);

      beaconNetDeviceContainer = nrHelper->InstallUeDevice (beaconNodeContainer, allBwps);
      DynamicCast<NrUeNetDevice> (beaconNetDeviceContainer.Get (0))->UpdateConfig ();

      nrSlHelper->PrepareUeForSidelink (beaconNetDeviceContainer, bwpIdContainer);
      nrSlHelper->InstallNrSlPreConfiguration (beaconNetDeviceContainer, slPreConfigNr);

      stream += nrHelper->AssignStreams (beaconNetDeviceContainer, stream);
      stream += nrSlHelper->AssignStreams (beaconNetDeviceContainer, stream);

      internet.Install (beaconNodeContainer);
      epcHelper->AssignUeIpv4Address (beaconNetDeviceContainer);
      Ptr<Ipv4StaticRouting> beaconStaticRouting = ipv4RoutingHelper.GetStaticRouting (beaconNode->GetObject<Ipv4> ());
      beaconStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      // Same group/dstL2Id as the vehicles above: vehicles' existing
      // RECEIVE bearer (just activated) already accepts this traffic, so
      // only the beacon's own TRANSMIT (and, for stack symmetry, RECEIVE)
      // direction needs activating here.
      Ptr<LteSlTft> beaconTftTx = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
      nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, beaconNetDeviceContainer, beaconTftTx);

      Ptr<LteSlTft> beaconTftRx = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
      nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, beaconNetDeviceContainer, beaconTftRx);

      // Socket created (and bound) here too, not in the post-SumoSetup
      // block below, for the same reason: Socket::CreateSocket must not
      // sit between vehicle construction and the per-vehicle application
      // installs that SumoSetup()/SumoSimulationStep() trigger later.
      beaconSocket = Socket::CreateSocket (beaconNode, UdpSocketFactory::GetTypeId ());
      beaconSocket->SetAllowBroadcast (true);
      beaconSocket->Bind (InetSocketAddress (Ipv4Address::GetAny (), port));
    }

  /*
   * Generic RSU nodes (--rsu-count, off by default: 0 nodes). Same
   * construction-order rule as the beacon block above and for the same
   * reason: placed here, strictly after all vehicle- and beacon-related
   * construction, so that a run with the default rsuCount==0 creates zero
   * additional nodes and consumes zero additional RNG draws -- provably
   * identical to a build that never had --rsu-count at all, regardless of
   * whether --beacon is also passed.
   *
   * Mirrors the beacon block's sequence (device install, sidelink prep,
   * pre-configuration, AssignStreams, IP stack, bearer activation, socket)
   * applied to rsuCount nodes at once via nrHelper/nrSlHelper's
   * container-based APIs, rather than duplicating per-node.
   */
  if (rsuCount > 0)
    {
      rsuNodeContainer.Create (rsuCount);
      MobilityHelper rsuMobility;
      rsuMobility.Install (rsuNodeContainer);

      rsuNetDeviceContainer = nrHelper->InstallUeDevice (rsuNodeContainer, allBwps);
      for (auto it = rsuNetDeviceContainer.Begin (); it != rsuNetDeviceContainer.End (); ++it)
        {
          DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
        }

      nrSlHelper->PrepareUeForSidelink (rsuNetDeviceContainer, bwpIdContainer);
      nrSlHelper->InstallNrSlPreConfiguration (rsuNetDeviceContainer, slPreConfigNr);

      stream += nrHelper->AssignStreams (rsuNetDeviceContainer, stream);
      stream += nrSlHelper->AssignStreams (rsuNetDeviceContainer, stream);

      internet.Install (rsuNodeContainer);
      epcHelper->AssignUeIpv4Address (rsuNetDeviceContainer);
      for (uint32_t r = 0; r < rsuNodeContainer.GetN (); ++r)
        {
          Ptr<Ipv4StaticRouting> rsuStaticRouting = ipv4RoutingHelper.GetStaticRouting (rsuNodeContainer.Get (r)->GetObject<Ipv4> ());
          rsuStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
        }

      // Same group/dstL2Id as the vehicles and the beacon: both TX and RX
      // activated for stack symmetry, same as the beacon's bearer above.
      Ptr<LteSlTft> rsuTftTx = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
      nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, rsuNetDeviceContainer, rsuTftTx);

      Ptr<LteSlTft> rsuTftRx = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
      nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, rsuNetDeviceContainer, rsuTftRx);

      // One socket per RSU, bound the same way beaconSocket is, so a later
      // task can transmit from RSU i via rsuSockets[i] independently of
      // any other RSU.
      rsuSockets.resize (rsuCount);
      for (uint32_t r = 0; r < rsuCount; ++r)
        {
          rsuSockets[r] = Socket::CreateSocket (rsuNodeContainer.Get (r), UdpSocketFactory::GetTypeId ());
          rsuSockets[r]->SetAllowBroadcast (true);
          rsuSockets[r]->Bind (InetSocketAddress (Ipv4Address::GetAny (), port));
        }
    }

  // enable log component
  //LogComponentEnable("NrUeMac", LOG_LEVEL_INFO);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (penetrationRate));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

  std::string sumo_additional_options = "--verbose true";

  if(sumo_netstate_file_name!="")
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (sumo_wait_for_socket)));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor prrSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &prrSupObj;
      metSup->setTraCIClient(sumoClient);
    }

  /*** 7. Setup interface and application for dynamic nodes ***/
  emergencyVehicleAlertHelper EmergencyVehicleAlertHelper;
  EmergencyVehicleAlertHelper.SetAttribute ("Client", PointerValue (sumoClient));
  EmergencyVehicleAlertHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  EmergencyVehicleAlertHelper.SetAttribute ("PrintSummary", BooleanValue (true));
  EmergencyVehicleAlertHelper.SetAttribute ("CSV", StringValue(csv_name));
  EmergencyVehicleAlertHelper.SetAttribute ("Model", StringValue ("nrv2x"));
  EmergencyVehicleAlertHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  /* callback function for node creation */
  int i=0;
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      if (nodeCounter >= allSlUesContainer.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = allSlUesContainer.Get(nodeCounter);
      ++nodeCounter; // increment counter for next node

      /* Install Application */
      EmergencyVehicleAlertHelper.SetAttribute ("IpAddr", Ipv4AddressValue(groupAddress4));
      i++;

      // EVA disabled for clean gossip PLR measurements.
      // Install() alone is not enough — ns-3 defaults start time to Seconds(0),
      // so the app would run anyway. The NR-V2X bearer is activated globally
      // before this lambda and does not depend on the EVA app being installed.
      //ApplicationContainer CAMSenderApp = CamSenderHelper.Install (includedNode);
      //ApplicationContainer AppSample = EmergencyVehicleAlertHelper.Install (includedNode);
      //AppSample.Start (Seconds (0.0));
      //AppSample.Stop (Seconds(simTime) - Simulator::Now () - Seconds (0.1));

      /* Install gossip relay application */
      V2xGossipAppHelper gossipHelper;
      gossipHelper.SetAttribute ("VehicleId", StringValue (vehicleID));
      gossipHelper.SetAttribute ("Port", UintegerValue (gossipPort));
      ApplicationContainer GossipApp = gossipHelper.Install (includedNode);
      GossipApp.Start (Seconds (0.0));
      GossipApp.Stop (Seconds(simTime) - Simulator::Now () - Seconds (0.1));
      sumoClient->RegisterGossipApp (vehicleID, GossipApp.Get(0));

      return includedNode;
    };

  /* callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
    {
      /* stop all applications */
      Ptr<V2xGossipApp> gossipApp = exNode->GetApplication(0)->GetObject<V2xGossipApp>();
      if(gossipApp)
        gossipApp->StopApplicationNow();

       /* set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0)); // rand() for visualization purposes
    };

  /* start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  // Frame size declaration (TASK 2): 1-byte UDP payload, constant size,
  // every beacon. +8 UDP / +20 IPv4 header bytes on the wire; NR SL
  // MAC/PHY framing (SCI, subheaders, channel coding) is on top of this
  // and depends on the run's MCS/resource allocation, so it is not
  // included here. Declared at this scope so both blocks below can see it.
  const uint32_t beaconPayloadBytes = 1;

  /*
   * TASK 1 (cont'd): register the beacon node as a station now that SUMO
   * is up, and place it at the live map center — read from TraCI's
   * getNetBoundary() (a value, not a hardcoded constant), so this works
   * unchanged on whatever map extent a run actually uses.
   */
  if (enableBeacon)
    {
      libsumo::TraCIPositionVector netBoundary = sumoClient->simulation.getNetBoundary ();
      double beaconX = (netBoundary[0].x + netBoundary[1].x) / 2.0;
      double beaconY = (netBoundary[0].y + netBoundary[1].y) / 2.0;
      DoubleValue altitudeValue;
      sumoClient->GetAttribute ("Altitude", altitudeValue);
      sumoClient->AddStation ("beacon0", beaconX, beaconY, altitudeValue.Get (), beaconNode);

      if (beaconNodeId < 0)
        {
          beaconNodeId = static_cast<int64_t> (beaconNode->GetId ());
        }

      const uint32_t beaconWireBytesPerBeacon = beaconPayloadBytes + 8 + 20;
      std::cout << "[nr-beacon] node=" << beaconNode->GetId ()
                << " payload=" << beaconPayloadBytes << "B"
                << " wire~=" << beaconWireBytesPerBeacon << "B/beacon"
                << " @1Hz => ~" << beaconWireBytesPerBeacon << " B/s"
                << " (IPv4+UDP headers included; SL MAC/PHY framing not included)"
                << std::endl;
    }

  /*
   * RSU placement (--rsu-count): once sumoClient exists (same point in
   * setup as the beacon's placement above), distribute the rsuCount nodes
   * over the SUMO network's bounding box (TraCI's getNetBoundary(), a live
   * value, same source the beacon uses for the map center) on a uniform
   * grid: cols = ceil(sqrt(N)), rows = ceil(N/cols), one node per grid
   * cell placed at the cell's center. A 2-D grid was chosen over spacing
   * nodes along a single line because the eventual broker use case needs
   * area coverage sampled across the whole map, not along one axis; a
   * grid is also the simplest deterministic (no RNG draws) scheme that
   * scales to arbitrary N without clustering nodes at the center the way
   * e.g. concentric rings would for small N.
   */
  if (rsuCount > 0)
    {
      libsumo::TraCIPositionVector rsuNetBoundary = sumoClient->simulation.getNetBoundary ();
      double minX = std::min (rsuNetBoundary[0].x, rsuNetBoundary[1].x);
      double maxX = std::max (rsuNetBoundary[0].x, rsuNetBoundary[1].x);
      double minY = std::min (rsuNetBoundary[0].y, rsuNetBoundary[1].y);
      double maxY = std::max (rsuNetBoundary[0].y, rsuNetBoundary[1].y);
      double width = maxX - minX;
      double height = maxY - minY;

      uint32_t cols = static_cast<uint32_t> (std::ceil (std::sqrt (static_cast<double> (rsuCount))));
      uint32_t rows = static_cast<uint32_t> (std::ceil (static_cast<double> (rsuCount) / cols));

      DoubleValue rsuAltitudeValue;
      sumoClient->GetAttribute ("Altitude", rsuAltitudeValue);

      for (uint32_t r = 0; r < rsuCount; ++r)
        {
          uint32_t gridRow = r / cols;
          uint32_t gridCol = r % cols;
          double rsuX = minX + (gridCol + 0.5) * (width / cols);
          double rsuY = minY + (gridRow + 0.5) * (height / rows);

          std::string rsuId = "rsu" + std::to_string (r);
          sumoClient->AddStation (rsuId, rsuX, rsuY, rsuAltitudeValue.Get (), rsuNodeContainer.Get (r));

          // Broker-over-sidelink dispatch (TASK 1): register this RSU's
          // socket under the same key it was just placed at in m_NodeMap,
          // in TraciClient's gossip dispatch map, so a broker envelope
          // addressed (after nearest-RSU selection) to this id transmits
          // from here. Targets gossipPort, not the local "port" (8000)
          // this socket happens to be bound to -- UDP send destination is
          // independent of the sender's own bound port, and gossipPort is
          // where vehicles' V2xGossipApp actually listens.
          sumoClient->RegisterRsuSend (rsuId, rsuSockets[r], groupAddress4, gossipPort);

          libsumo::TraCIPosition rsuLonLat = sumoClient->simulation.convertXYtoLonLat (rsuX, rsuY);
          std::cerr << "[nr-rsu] index=" << r
                     << " node=" << rsuNodeContainer.Get (r)->GetId ()
                     << " x=" << rsuX << " y=" << rsuY
                     << " lon=" << rsuLonLat.x << " lat=" << rsuLonLat.y
                     << std::endl;
        }
    }

  /*
   * TASK 3/4: enable the decode-coverage counter/emitter. Deliberately a
   * separate condition from enableBeacon: this is what lets the same
   * counting code "work unchanged once the broker node is added" (run
   * with only --beacon-node-id=<realBrokerId>, no --beacon, once a real
   * broker exists elsewhere in the scenario). Must run before the first
   * ScheduleSlBeaconTx() call below, since that call's
   * NrSlBeaconCoverageStartNewBeacon() is a no-op until Enable() has run.
   */
  if (beaconNodeId >= 0)
    {
      NrSlBeaconCoverageEnable (static_cast<uint32_t> (beaconNodeId));

      sumoClient->SetPerTickCallback ([] (const std::vector<uint32_t>& liveVehicleNodeIds)
        {
          NrSlBeaconCoverageSnapshot snap = NrSlBeaconCoverageGetSnapshot ();

          // Decoders are node ids, not yet filtered to "currently live
          // vehicle" (see nr-sl-beacon-coverage.h): the vehicle pool has
          // up to numberOfNodes nodes with an active sidelink stack from
          // t=0, but only a subset are claimed/live in m_NodeMap at any
          // tick — an unclaimed or already-parked pool node can still
          // physically decode a broadcast it's in range of. Intersect
          // against the live set so decoded can never exceed live.
          std::unordered_set<uint32_t> liveSet (liveVehicleNodeIds.begin (), liveVehicleNodeIds.end ());
          uint32_t decodedAmongLive = 0;
          for (uint32_t nodeId : snap.decoderNodeIds)
            {
              if (liveSet.count (nodeId) > 0)
                {
                  ++decodedAmongLive;
                }
            }

          uint32_t liveCount = static_cast<uint32_t> (liveVehicleNodeIds.size ());
          double frac = liveCount > 0 ? static_cast<double> (decodedAmongLive) / liveCount : 0.0;
          std::cerr << "[nr-beacon-coverage] t=" << Simulator::Now ().GetSeconds ()
                     << " seq=" << snap.seq
                     << " decoded=" << decodedAmongLive
                     << " live=" << liveCount
                     << " frac=" << frac
                     << std::endl;
        });
    }

  // TASK 2: arm the 1 Hz coverage-probe beacon transmission. beaconSocket
  // was already created and bound in the consolidated pre-SumoSetup block
  // above (see the RNG-ordering comment there); only the first
  // transmission is kicked off here, after coverage-counting is enabled.
  if (enableBeacon)
    {
      ScheduleSlBeaconTx (beaconSocket, groupAddress4, port, beaconPayloadBytes);
    }

  /* Send static polygon overlays (parking spots, H3 cells, ...) to the visualizer.
   * SumoSetup() has already called sendMapDraw(), so sendPolygonUpdate() is safe to call.
   * We read the <additional-files> list from the .sumo.cfg that was already provided,
   * so no extra argument is needed — it just works automatically. */
  if (vehicle_vis)
    {
      // Resolve the directory containing the .sumo.cfg to use as base for relative paths
      std::string cfg_dir = sumo_config;
      auto last_slash = cfg_dir.find_last_of ("/\\");
      cfg_dir = (last_slash != std::string::npos) ? cfg_dir.substr (0, last_slash + 1) : "./";

      // Helper: parse one .add.xml file and send every <poly> to the visualizer
      auto send_polys_from_file = [&](const std::string &add_path)
        {
          std::cout << "[poly] parsing: " << add_path << std::endl;
          xmlDocPtr doc = xmlParseFile (add_path.c_str ());
          if (doc == nullptr)
            {
              std::cout << "[poly] ERROR: could not open/parse: " << add_path << std::endl;
              return;
            }
          int poly_count = 0;

          xmlNodePtr root = xmlDocGetRootElement (doc);
          for (xmlNodePtr n = root->children; n != nullptr; n = n->next)
            {
              if (n->type != XML_ELEMENT_NODE) continue;
              if (xmlStrcmp (n->name, BAD_CAST "poly") != 0) continue;

              xmlChar *xid    = xmlGetProp (n, BAD_CAST "id");
              xmlChar *xcolor = xmlGetProp (n, BAD_CAST "color");
              xmlChar *xshape = xmlGetProp (n, BAD_CAST "shape");

              if (xid && xcolor && xshape)
                {
                  uint8_t cr = 0, cg = 0, cb = 0, ca = 255;
                  int nparsed = sscanf (reinterpret_cast<const char *>(xcolor),
                                        "%hhu,%hhu,%hhu,%hhu", &cr, &cg, &cb, &ca);
                  if (nparsed >= 3)
                    {
                      auto coords = vehicleVisualizer::parseSumoShape (
                          reinterpret_cast<const char *>(xshape));

                      // The parking polygon generator outputs shapes in geographic (lon,lat)
                      // format — no TraCI conversion needed here.
                      // parseSumoShape() already stores pairs as (lon, lat) which is what
                      // sendPolygonUpdate() and the JS client expect.
                      if (poly_count == 0 && !coords.empty ())
                        std::cout << "[poly] first vertex: lon=" << coords[0].first
                                  << " lat=" << coords[0].second << std::endl;

                      vehicleVis->sendPolygonUpdate (
                          reinterpret_cast<const char *>(xid), cr, cg, cb, ca, coords);
                      ++poly_count;
                      // Yield every 100 sends so the OS can drain the Node.js UDP receive buffer.
                      if (poly_count % 100 == 0) usleep (1000);
                    }
                  else
                    std::cout << "[poly] WARNING: bad color '" << xcolor
                              << "' for poly '" << xid << "'" << std::endl;
                }
              if (xid)    xmlFree (xid);
              if (xcolor) xmlFree (xcolor);
              if (xshape) xmlFree (xshape);
            }
          std::cout << "[poly] sent " << poly_count << " polygons from " << add_path << std::endl;
          xmlFreeDoc (doc);
        };

      // Open the .sumo.cfg and find <additional-files value="..."/>
      std::cout << "[poly] reading sumo config: " << sumo_config << std::endl;
      std::cout << "[poly] cfg_dir resolved to: " << cfg_dir << std::endl;
      xmlDocPtr cfg_doc = xmlParseFile (sumo_config.c_str ());
      if (cfg_doc != nullptr)
        {
          xmlNodePtr cfg_root = xmlDocGetRootElement (cfg_doc);
          for (xmlNodePtr sect = cfg_root->children; sect != nullptr; sect = sect->next)
            {
              if (sect->type != XML_ELEMENT_NODE) continue;
              if (xmlStrcmp (sect->name, BAD_CAST "input") != 0) continue;
              for (xmlNodePtr field = sect->children; field != nullptr; field = field->next)
                {
                  if (field->type != XML_ELEMENT_NODE) continue;
                  if (xmlStrcmp (field->name, BAD_CAST "additional-files") != 0) continue;
                  xmlChar *xval = xmlGetProp (field, BAD_CAST "value");
                  if (!xval) continue;

                  // value may be a comma-separated list of files
                  std::istringstream ss (reinterpret_cast<const char *>(xval));
                  std::string token;
                  while (std::getline (ss, token, ','))
                    {
                      // trim spaces
                      token.erase (0, token.find_first_not_of (" \t"));
                      token.erase (token.find_last_not_of (" \t") + 1);
                      if (token.empty ()) continue;
                      // resolve relative to the cfg directory
                      std::string full = (token[0] == '/') ? token : cfg_dir + token;
                      send_polys_from_file (full);
                    }
                  xmlFree (xval);
                }
            }
          xmlFreeDoc (cfg_doc);
        }
      else
        NS_LOG_WARN ("vehicle-visualizer: could not open sumo config to read additional-files: " << sumo_config);
    }

  /*** 8. Start Simulation ***/
  Simulator::Stop (Seconds(simTime));

  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
    {
      if(csv_name_cumulative!="")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK)!=-1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,avg_PRR,avg_latency_ms" << std::endl;
        }

        csv_cum_ofstream << txPower << "," << metSup->getAveragePRR_overall () << "," << metSup->getAverageLatency_overall () << std::endl;
      }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
    }


  return 0;
}


