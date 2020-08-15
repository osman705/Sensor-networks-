/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/aodv-module.h"
using namespace ns3;

uint32_t bytesTotal;
uint32_t packetsReceived;
uint32_t Node0Pending = 0;
uint32_t Node1SendAck = 0;
uint32_t Node1Pending = 0;
uint32_t Node2SendAck = 0;
uint16_t Pkt_no_last_seen_by_node2 = 0;
uint16_t Pkt_no_being_sent_by_node1 = 0;
uint16_t Pkt_no_last_seen_by_node1 = 0;
uint16_t Pkt_no_being_sent_by_node0 = 1;
uint16_t countMail = 0;

uint32_t capacity = 3;
uint16_t head = 0;
uint16_t tail = 0;
uint16_t globalcounter = 1;
uint16_t isbufferempty = 1;
NS_LOG_COMPONENT_DEFINE ("FinalProject");


uint16_t node1_head = 0;
uint16_t node1_tail = 0;
uint16_t node1_globalcounter = 0;
uint16_t node1_isbufferempty = 1;


//----Experiment parameters

double distance_between_node0_node2 = 10000; //x-axis distance
double distance_between_node0_node1 = 3; //y-axis distance
double node1velocity = 20; //node 1 velocity in x-axis (m/s)

double duration = 500;

//--------------------Custom header code begin------------//

class MyHeader : public Header
{
public:
  // new methods
  void SetData (uint16_t data);
  uint16_t GetData (void);
  void SetPacketType (uint16_t data);
  uint16_t GetPacketType (void);
  void Setisbufferempty (uint16_t data);
  uint16_t Getisbufferempty (void);
  void Sethead (uint16_t data);
  uint16_t Gethead (void);
  void Settail (uint16_t data);
  uint16_t Gettail (void);
  void Setglobalcounter (uint16_t data);
  uint16_t Getglobalcounter (void);
  // new method needed
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  // overridden from Header
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;
private:
  uint16_t m_data;
  uint16_t m_packettype;
  uint16_t m_isbufferempty;
  uint16_t m_head;
  uint16_t m_tail;
  uint16_t m_globalcounter;
};

//Implementation of public members of class MyHeader
void
MyHeader::SetData (uint16_t data)
{
  m_data = data;
}

uint16_t
MyHeader::GetData (void)
{
  return m_data;
}

void
MyHeader::SetPacketType (uint16_t data)
{
  m_packettype = data;
}

uint16_t
MyHeader::GetPacketType (void)
{
  return m_packettype;
}

void
MyHeader::Setisbufferempty (uint16_t data)
{
  m_isbufferempty = data;
}

uint16_t
MyHeader::Getisbufferempty (void)
{
  return m_isbufferempty;
}

void
MyHeader::Sethead (uint16_t data)
{
  m_head = data;
}

uint16_t
MyHeader::Gethead (void)
{
  return m_head;
}

void
MyHeader::Settail (uint16_t data)
{
  m_tail = data;
}

uint16_t
MyHeader::Gettail (void)
{
  return m_tail;
}

void
MyHeader::Setglobalcounter (uint16_t data)
{
  m_globalcounter = data;
}

uint16_t
MyHeader::Getglobalcounter (void)
{
  return m_globalcounter;
}

uint32_t
MyHeader::GetSerializedSize (void) const
{
 // two bytes of data to store
//  return 2;
//  return 4;
    return 12;
}

void
MyHeader::Serialize (Buffer::Iterator start) const
{
  start.WriteHtonU16 (m_data);
  start.WriteHtonU16 (m_packettype);
  start.WriteHtonU16 (m_isbufferempty);
  start.WriteHtonU16 (m_head);
  start.WriteHtonU16 (m_tail);
  start.WriteHtonU16 (m_globalcounter);
}

uint32_t
MyHeader::Deserialize (Buffer::Iterator start)
{
  m_data = start.ReadNtohU16 ();
  m_packettype = start.ReadNtohU16 ();
  m_isbufferempty = start.ReadNtohU16 ();
  m_head = start.ReadNtohU16 ();
  m_tail = start.ReadNtohU16 ();
  m_globalcounter = start.ReadNtohU16 ();
  return 12;
}

void
MyHeader::Print (std::ostream &os) const
{
  os << m_data;
}

TypeId
MyHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MyHeader")
    .SetParent<Header> ()
    .AddConstructor<MyHeader> ()
    ;
  return tid;
}

TypeId
MyHeader::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
//--------------------Custom header code end------------//

//This function keeps track of the position of a moving node using (x,y) coordinates
void CourseChangeSink (Ptr<OutputStreamWrapper> stream, std::string context, Ptr<const MobilityModel> model)
{
        Vector position = model->GetPosition ();
        NS_LOG_UNCOND (Simulator::Now ().GetSeconds () <<
        " Course change! x = " << position.x << ", y = " << position.y);
        *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << position.x << "\t" << position.y << std::endl;
}


//---Higher level code for Node 0 BEGIN

static void Node0DataGen ()
{
//      NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 0 generating data.");
        if (isbufferempty == 1)
        {
//              NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 0 buffer is empty.");
                head = globalcounter;
                tail = globalcounter;
                isbufferempty = 0;
        }
        else
        {
//              NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 0 buffer is NOT empty.");
                if (((head - tail) + 1) >= (signed int)capacity)
                {
//                      NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 0 buffer is full - must remove elements");
                        tail = tail + 1;
                        head = head + 1;
                }
                else
                {
//                      NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 0 buffer is NOT full.");
                        head = head + 1;
                }
        }
        globalcounter = globalcounter + 1;
        Simulator::Schedule (Seconds(1.00), &Node0DataGen);
}

static void Node0SendPacket (Ptr<Socket> socket, uint32_t pktSize)
{
        Ptr<Packet> p = Create<Packet> ();
        MyHeader XXheader;
        if (isbufferempty == 0)
        {
//             NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 0 sending a packet.");
               XXheader.SetPacketType (1); // 1 for data
               XXheader.Setisbufferempty (isbufferempty);  // header fields
               XXheader.Sethead (head);  // header fields
               XXheader.Settail (tail);  // header fields
               XXheader.Setglobalcounter (globalcounter);  // header fields
               p->AddHeader (XXheader);
               socket->Send (p);
        }
        else
        {
//               NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 0 has nothing to send.");
        }
        Simulator::Schedule (Seconds(0.25), &Node0SendPacket, socket, pktSize);
}


static void Node0ReceivePacket (Ptr<OutputStreamWrapper> stream, Ptr<Socket> socket)
{
        NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 0 received a packet.");
        isbufferempty = 1;
}

//---Higher level code for node 0 END



//---Higher level code for node 1 BEGIN

//Advanced declaration
static void Node1SendPacket (Ptr<Socket> socket, uint32_t pktSize);

static void Node1AckLoop (Ptr<Socket> socket, Ptr<Socket> socket2, uint32_t pktSize)
{
        if (Node1SendAck == 1)
        {
                NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 1 sending an ACK.");
                Node1SendAck = 0;
                socket->Send (Create<Packet> (pktSize));
                Node1Pending = 1;
                Simulator::Schedule (Seconds (0.0001), &Node1SendPacket, socket2, pktSize);
        }
        Simulator::Schedule (Seconds(0.01), &Node1AckLoop, socket, socket2, pktSize);
}

static void Node1ReceivePacket (Ptr<OutputStreamWrapper> stream, Ptr<Socket> socket)
{
//      NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 1 received a packet. UNIFIED receiver.");

        MyHeader header;
        uint16_t type_of_packet;
        Ptr<Packet> packet;
        packet = socket->Recv();
        packet->RemoveHeader(header);
        type_of_packet = header.GetPacketType();
        Pkt_no_being_sent_by_node0 = header.Getglobalcounter ();
        if (type_of_packet == 1) //data
        {
                if (Pkt_no_last_seen_by_node1 < Pkt_no_being_sent_by_node0)
                {
                        NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " The packet received by 1 is a *UNIQUE* DATA packet.");
                        Pkt_no_last_seen_by_node1 = Pkt_no_being_sent_by_node0;
                        Pkt_no_being_sent_by_node1 = Pkt_no_last_seen_by_node1;
                        Node1SendAck = 1;
                        // strip packet contents and transfer contents to variables
                        node1_isbufferempty = header.Getisbufferempty ();  // header fields
                        node1_head = header.Gethead ();  // header fields
                        node1_tail = header.Gettail ();  // header fields
                        node1_globalcounter = header.Getglobalcounter ();  // header fields
                }
        }
        else
        {
                NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " The packet received by 1 is an ACK packet.");
                Node1Pending = 0;
        }


}


static void Node1SendPacket (Ptr<Socket> socket, uint32_t pktSize)
{
        Ptr<Packet> p = Create<Packet> ();
        MyHeader XXheader;
        if (Node1Pending == 1)
        {
//              NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << "Node 1 sending a packet.");
                XXheader.SetPacketType (1); // 1 for data
                XXheader.Setisbufferempty (node1_isbufferempty);  // header fields
                XXheader.Sethead (node1_head);  // header fields
                XXheader.Settail (node1_tail);  // header fields
                XXheader.Setglobalcounter (node1_globalcounter);  // header fields
                p->AddHeader (XXheader);
                socket->Send (p);
                Simulator::Schedule (Seconds(0.25), &Node1SendPacket, socket, pktSize);
        }
}

//---Higher level code for node 1 END


//---Higher level code for node 2 BEGIN

static void Node2ReceivePacket (Ptr<OutputStreamWrapper> stream, Ptr<Socket> socket)
{
        uint16_t receivedpacket_isbufferempty;
        uint16_t receivedpacket_head;
        uint16_t receivedpacket_tail;
        uint16_t receivedpacket_globalcounter;
        Ptr<Packet> packet;
        MyHeader header;
        uint16_t newMails = 0;

        packet = socket->Recv();
        packet->RemoveHeader(header);
        receivedpacket_isbufferempty = header.Getisbufferempty();
        receivedpacket_head = header.Gethead();
        receivedpacket_tail = header.Gettail();
        receivedpacket_globalcounter = header.Getglobalcounter();

        if (Pkt_no_last_seen_by_node2 < receivedpacket_globalcounter)
        {
                Pkt_no_last_seen_by_node2 = receivedpacket_globalcounter;
                NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 2 received a packet.");
                std::cout << "*------------*" <<  std::endl;
                std::cout << "The value of *isbufferempty* is " << receivedpacket_isbufferempty << std::endl;
                std::cout << "The value of *head* is " << receivedpacket_head << std::endl;
                std::cout << "The value of *tail* is " << receivedpacket_tail << std::endl;
                std::cout << "The value of *globalcounter* is " << receivedpacket_globalcounter << std::endl;
                std::cout << "*------------*" <<  std::endl;

                newMails = receivedpacket_head - receivedpacket_tail + 1;
                countMail = countMail + newMails;
                std::cout << "The number of mails received by Node 2 so far is " << countMail << std::endl;

                Node2SendAck = 1;
        }
}

static void Node2AckLoop (Ptr<Socket> socket, uint32_t pktSize)
{
        Ptr<Packet> p = Create<Packet> ();
        MyHeader XXheader;
        if (Node2SendAck == 1)
        {
//              Pkt_no_last_seen_by_node2 = Pkt_no_being_sent_by_node1;
                NS_LOG_UNCOND(Simulator::Now ().GetSeconds () << " Node 2 sending an ACK.");
                Node2SendAck = 0;
                XXheader.SetPacketType (0); // 0 for ack
                p->AddHeader (XXheader);
                socket->Send (p);
        }
        Simulator::Schedule (Seconds(0.01), &Node2AckLoop, socket, pktSize);
}

//---Higher level code for node 2 END


//This is the main function
int main (int argc, char *argv[])
{

        double txp = 1.5; //transmission power dB
        std::string phyMode ("DsssRate1Mbps");

        //Set Non-unicastMode rate to unicast mode
        Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

        // 1. Create the nodes
        NodeContainer adhocNodes;
        adhocNodes.Create(3);  //All 3 nodes are members of adhocNodes
        NodeContainer mobileNode = NodeContainer (adhocNodes.Get(1)); //Node 1 is defined as the mobileNode (i.e., the bus)
        NodeContainer stationaryNodes = NodeContainer (adhocNodes.Get(0), adhocNodes.Get(2)); //Node 0 and Node 2 are defined as stationaryNodes (i.e., the villages)

        // 2. Set up physical layer
        WifiHelper wifi;
        wifi.SetStandard (WIFI_PHY_STANDARD_80211b); //Use 802.11 standard
        YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
        wifiPhy.Set ("RxGain", DoubleValue (-10) ); //Set up the gain at the receiver (in dB)

        // 3. Set up propagation loss model
        YansWifiChannelHelper wifiChannel;

        std::string lossModel = "ns3::RangePropagationLossModel"; //Range propagation loss model
        std::string atr1 = "MaxRange";
        wifiChannel.AddPropagationLoss (lossModel, atr1, DoubleValue(200)); // maximum transmission range = 200 m

        wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel"); //set up the propagation delay model
        wifiPhy.SetChannel (wifiChannel.Create ());

        wifiPhy.Set ("TxPowerStart",DoubleValue (txp)); //set up the transmission power
        wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));


        // 4. Set up MAC layer and install wireless devices
        NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
        wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue ("DsssRate2Mbps"),
                                      "ControlMode",StringValue ("DsssRate1Mbps"));
        wifiMac.SetType ("ns3::AdhocWifiMac"); //Set MAC to ad hoc mode
        NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, adhocNodes);

        // 5. Set up mobility model for Node 1
        MobilityHelper mobilityMobileNode;

        Ptr<ListPositionAllocator> positionAllocMobileNode = CreateObject<ListPositionAllocator> ();
        positionAllocMobileNode->Add (Vector (0.0, distance_between_node0_node1, 0.0)); //3D coordinates (x,y,z). Use only the x-y plane
        mobilityMobileNode.SetPositionAllocator (positionAllocMobileNode);

        mobilityMobileNode.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
        mobilityMobileNode.Install (mobileNode);

        Ptr<ConstantVelocityMobilityModel> mob = mobileNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();
        mob->SetVelocity(Vector(node1velocity, 0, 0)); //velocity 20 m/s in x-axis

        // 6. Connect trace source to trace sink for Node 1
        std::ostringstream oss;
        oss <<
        "/NodeList/" << mobileNode.Get (0)->GetId () <<
        "/$ns3::MobilityModel/CourseChange";

        AsciiTraceHelper asciiTraceHelper;
        Ptr<OutputStreamWrapper> locationStream = asciiTraceHelper.CreateFileStream ("Location.txt");

        Config::Connect (oss.str (), MakeBoundCallback (&CourseChangeSink, locationStream));

        // 7. Assign positions to Node 0 and Node 2
        MobilityHelper mobilityStaNodes;
        mobilityStaNodes.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> positionAllocStaNodes = CreateObject<ListPositionAllocator> ();
        positionAllocStaNodes->Add (Vector (0.0, 0.0, 0.0)); //3D coordinates (x,y,z). Use only the x-y plane
        positionAllocStaNodes->Add (Vector (distance_between_node0_node2, 0.0, 0.0)); //3D coordinates (x,y,z). Use only the x-y plane
        mobilityStaNodes.SetPositionAllocator(positionAllocStaNodes);
        mobilityStaNodes.Install (stationaryNodes);

        // 8. Set up  the routing protocol (AODV)
        AodvHelper aodv;
        Ipv4ListRoutingHelper list;
        InternetStackHelper internet;
        aodv.Set("EnableHello", BooleanValue(false));
        aodv.Set("GratuitousReply", BooleanValue(false));
        aodv.Set("ActiveRouteTimeout", ns3::TimeValue(Seconds(600)));
        list.Add (aodv, 100);

        internet.SetRoutingHelper (list);
        internet.Install (adhocNodes);

        // 9. Assign IP addresses to nodes
        Ipv4AddressHelper address;
        address.SetBase ("10.1.1.0", "255.255.255.0");
        Ipv4InterfaceContainer interfaces;
        interfaces = address.Assign (devices);

        // 10. Receive sides here; we connect the receiving sockets of each node to their respective callbacks
        // These callbacks basically define what the node does next upon receiving a packet

        //---node 0's receive side BEGIN
        TypeId tid0 = TypeId::LookupByName ("ns3::UdpSocketFactory");
        Ptr<Socket> Node0Recv = Socket::CreateSocket (adhocNodes.Get (0), tid0);
        InetSocketAddress local0 = InetSocketAddress (Ipv4Address::GetAny (), 9);
        Node0Recv->Bind (local0);
        Ptr<OutputStreamWrapper> Node0rcvdStream = asciiTraceHelper.CreateFileStream ("Node0_Rcvd.txt");
        Node0Recv->SetRecvCallback (MakeBoundCallback (&Node0ReceivePacket, Node0rcvdStream));
        //---node 0's receive side END

        //---node 1's receive side BEGIN
        TypeId tid1 = TypeId::LookupByName ("ns3::UdpSocketFactory");
        Ptr<Socket> Node1Recv = Socket::CreateSocket (adhocNodes.Get (1), tid1);
        InetSocketAddress local1 = InetSocketAddress (Ipv4Address::GetAny (), 9);
        Node1Recv->Bind (local1);
        Ptr<OutputStreamWrapper> Node1rcvdStream = asciiTraceHelper.CreateFileStream ("Node1_Rcvd.txt");
        Node1Recv->SetRecvCallback (MakeBoundCallback (&Node1ReceivePacket, Node1rcvdStream));
        //---node 1's receive side  END

        //---node 2's receive side BEGIN
        TypeId tid2 = TypeId::LookupByName ("ns3::UdpSocketFactory");
        Ptr<Socket> Node2Recv = Socket::CreateSocket (adhocNodes.Get (2), tid2);
        InetSocketAddress local2 = InetSocketAddress (Ipv4Address::GetAny (), 9);
        Node2Recv->Bind (local2);
        Ptr<OutputStreamWrapper> Node2rcvdStream = asciiTraceHelper.CreateFileStream ("Node2_Rcvd.txt");
        Node2Recv->SetRecvCallback (MakeBoundCallback (&Node2ReceivePacket, Node2rcvdStream));
        //---node 2's receive side END


        // 11. Generate traffic
        uint32_t packetSize = 200; //bytes

	//Start the node 0 ---> node 1 connection
	//fire up Node0SendPacket
        Ptr<Socket> source = Socket::CreateSocket (adhocNodes.Get (0), tid0);
        InetSocketAddress SocketAddressof1 = InetSocketAddress (interfaces.GetAddress (1, 0), 9);
        source->Connect (SocketAddressof1);
        Node0Pending = 1;
        Simulator::Schedule (Seconds (0.0001), &Node0SendPacket, source, packetSize);

	//Start node 1's ACK loop (node 1 ---> node 0, socket 9)
	//We define the node 1---> node 2 connection here as well (socket 9)
	//We need 2 sockets: the first one is used by Node 1 to send ACKs to Node 0
	//The second one is used by Node 1 the send the packet to Node 2
	//Fire up Node1AckLoop
        Ptr<Socket> sourceSocketforNode1 = Socket::CreateSocket (adhocNodes.Get (1), tid1);
        InetSocketAddress SocketAddressof0 = InetSocketAddress (interfaces.GetAddress (0, 0), 9);
        sourceSocketforNode1->Connect (SocketAddressof0);

        Ptr<Socket> sourceSocketforNode1SECOND = Socket::CreateSocket (adhocNodes.Get (1), tid1);
        InetSocketAddress SocketAddressof2 = InetSocketAddress (interfaces.GetAddress (2, 0), 9);
        sourceSocketforNode1SECOND->Connect (SocketAddressof2);

        Simulator::Schedule (Seconds (0.0001), &Node1AckLoop, sourceSocketforNode1, sourceSocketforNode1SECOND, packetSize);

	//Start node 2's ACK loop (node 2---> node 1, socket 9)
	//Fire up Node2AckLoop
        Ptr<Socket> sourceSocketforNode2 = Socket::CreateSocket (adhocNodes.Get (2), tid2);
        sourceSocketforNode2->Connect (SocketAddressof1);
        Simulator::Schedule (Seconds (0.0001), &Node2AckLoop, sourceSocketforNode2, packetSize);

//        wifiPhy.EnableAsciiAll(asciiTraceHelper.CreateFileStream ("PacketsContent.txt"));

        Simulator::Schedule (Seconds(0.0001), &Node0DataGen);

        // 12. Run the simulation
        Simulator::Stop (Seconds (duration));
        Simulator::Run ();
        Simulator::Destroy ();

        return 0;
}
