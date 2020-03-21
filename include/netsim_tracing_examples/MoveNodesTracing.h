#include <uwsim/NetSim.h>

namespace netsim_tracing_examples {

using namespace uwsim;

class MoveNodesTracing : public NetSimTracing {
public:
  MoveNodesTracing();
  void Configure();
  void DoRun();

private:
  void PacketTransmitting(std::string path, ROSCommsDevicePtr dev,
                          ns3ConstPacketPtr pkt);
  void PacketError(std::string path, ROSCommsDevicePtr dev,
                   ns3ConstPacketPtr pkt, bool propError, bool colError);
  void PacketReceived(std::string path, ROSCommsDevicePtr dev,
                      ns3ConstPacketPtr pkt);

  ros::NodeHandle node;
  ros::Publisher bluerov0Pub, bluerov1Pub;
  dccomms::Timer showDistanceTimer;
};
} // namespace netsim_tracing_examples
