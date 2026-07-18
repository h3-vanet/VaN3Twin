#include "v2x-gossip-app.h"

#include <iostream>
#include "ns3/log.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/inet-socket-address.h"
#include "ns3/packet.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("V2xGossipApp");
NS_OBJECT_ENSURE_REGISTERED(V2xGossipApp);

TypeId
V2xGossipApp::GetTypeId()
{
  static TypeId tid =
      TypeId("ns3::V2xGossipApp")
      .SetParent<Application>()
      .SetGroupName("Applications")
      .AddConstructor<V2xGossipApp>()
      .AddAttribute("VehicleId",
                    "SUMO vehicle identifier",
                    StringValue(""),
                    MakeStringAccessor(&V2xGossipApp::m_vehicleId),
                    MakeStringChecker())
      .AddAttribute("Port",
                    "UDP port for gossip traffic",
                    UintegerValue(8001),
                    MakeUintegerAccessor(&V2xGossipApp::m_port),
                    MakeUintegerChecker<uint16_t>());
  return tid;
}

V2xGossipApp::V2xGossipApp() = default;
V2xGossipApp::~V2xGossipApp() = default;

void
V2xGossipApp::SetReceiveCallback(RxCallback cb)
{
  m_rxCallback = std::move(cb);
}

void
V2xGossipApp::StartApplication()
{
  m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  m_socket->SetAllowBroadcast(true);

  InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
  m_socket->Bind(local);

  m_socket->SetRecvCallback(MakeCallback(&V2xGossipApp::Receive, this));
}

void
V2xGossipApp::StopApplication()
{
  if (m_socket)
    {
      // Stop RX immediately, but defer Close() by one event cycle: an
      // Ipv4EndPoint::DoForwardUp already scheduled in this same simulation
      // step would otherwise run on a deallocated endpoint (use-after-free).
      m_socket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
      m_socket->ShutdownRecv();
      Ptr<Socket> sock = m_socket;
      Simulator::Schedule(MilliSeconds(1), [sock]() { sock->Close(); });
      m_socket = nullptr;
    }
}

void
V2xGossipApp::StopApplicationNow()
{
  StopApplication();
}

void
V2xGossipApp::Send(const uint8_t* data, uint32_t len)
{
  if (!m_socket) return;
  Ptr<Packet> pkt = Create<Packet>(data, len);
  InetSocketAddress dest(Ipv4Address("225.0.0.0"), m_port);
  m_socket->SendTo(pkt, 0, dest);
}

void
V2xGossipApp::Receive(Ptr<Socket> socket)
{
  Ptr<Packet> pkt;
  Address from;
  while ((pkt = socket->RecvFrom(from)) != nullptr)
    {
      uint32_t size = pkt->GetSize();
      static uint8_t buf[65536];
      uint32_t safe_size = std::min(size, static_cast<uint32_t>(sizeof(buf)));
      pkt->CopyData(buf, safe_size);
      if (m_rxCallback)
        m_rxCallback(m_vehicleId, buf, safe_size);
    }
}

} // namespace ns3
