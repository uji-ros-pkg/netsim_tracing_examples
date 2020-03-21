#include <uwsim/NetSim.h>

namespace netsim_tracing_examples {

using namespace uwsim;

class LogCommsTracing : public NetSimTracing {
public:
  LogCommsTracing();
  void Configure();

  void PacketTransmitting(std::string path, ROSCommsDevicePtr dev,
                          ns3ConstPacketPtr pkt);
  void PacketError(std::string path, ROSCommsDevicePtr dev,
                   ns3ConstPacketPtr pkt, bool propError, bool colError);
  void PacketReceived(std::string path, ROSCommsDevicePtr dev,
                      ns3ConstPacketPtr pkt);
};
}
