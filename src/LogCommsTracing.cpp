#include <class_loader/multi_library_class_loader.hpp>
#include <netsim_tracing_examples/LogCommsTracing.h>
#include <geometry_msgs/TwistStamped.h>

namespace netsim_tracing_examples {

LogCommsTracing::LogCommsTracing() : NetSimTracing() {}

void LogCommsTracing::PacketTransmitting(std::string path,
                                         ROSCommsDevicePtr dev,
                                         ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] TX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
}

void LogCommsTracing::PacketError(std::string path, ROSCommsDevicePtr dev,
                                  ns3ConstPacketPtr pkt, bool propErr,
                                  bool colErr) {

  NetsimHeader header;
  pkt->PeekHeader(header);
  if (propErr) {
    if (!colErr) {
      Warn("[{}] PERR -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
           dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
           header.GetPacketSize());
    } else {

      Warn("[{}] MERR -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
           dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
           header.GetPacketSize());
    }
  } else {

    Warn("[{}] COL -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
         dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
         header.GetPacketSize());
  }
}

void LogCommsTracing::PacketReceived(std::string path, ROSCommsDevicePtr dev,
                                     ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] RX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
}

void LogCommsTracing::Configure() {
  SetLogName("CommsLogger");

  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("%D %T.%F %v"));

  ns3::Config::Connect("/ROSDeviceList/*/PacketError",
                       ns3::MakeCallback(&LogCommsTracing::PacketError, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketReceived",
      ns3::MakeCallback(&LogCommsTracing::PacketReceived, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketTransmitting",
      ns3::MakeCallback(&LogCommsTracing::PacketTransmitting, this));
}

CLASS_LOADER_REGISTER_CLASS(LogCommsTracing, NetSimTracing)
} // namespace netsim_tracing_examples
