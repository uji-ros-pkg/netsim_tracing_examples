#include <class_loader/multi_library_class_loader.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <netsim_tracing_examples/LogMacCommsTracing.h>

namespace netsim_tracing_examples {

LogMacCommsTracing::LogMacCommsTracing() : NetSimTracing() {
  bluerov0Pub = node.advertise<geometry_msgs::TwistStamped>(
      "/bluerov0/velocityCommand", 1);
  bluerov1Pub = node.advertise<geometry_msgs::TwistStamped>(
      "/bluerov1/velocityCommand", 1);
}

void LogMacCommsTracing::PacketTransmitting(std::string path,
                                          ROSCommsDevicePtr dev,
                                          ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] TX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
}

void LogMacCommsTracing::PacketError(std::string path, ROSCommsDevicePtr dev,
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

void LogMacCommsTracing::PacketReceived(std::string path, ROSCommsDevicePtr dev,
                                      ns3ConstPacketPtr pkt) {
  NetsimHeader header;
  pkt->PeekHeader(header);
  Info("[{}] RX -- ID: {} ; MAC: {} ; Seq: {} ; Size: {}", path,
       dev->GetDccommsId(), dev->GetMac(), header.GetSeqNum(),
       header.GetPacketSize());
}

void LogMacCommsTracing::Configure() {
  SetLogName("CommsLogger");

  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("%D %T.%F %v"));

  ns3::Config::Connect("/ROSDeviceList/*/PacketError",
                       ns3::MakeCallback(&LogMacCommsTracing::PacketError, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketReceived",
      ns3::MakeCallback(&LogMacCommsTracing::PacketReceived, this));

  ns3::Config::Connect(
      "/ROSDeviceList/*/PacketTransmitting",
      ns3::MakeCallback(&LogMacCommsTracing::PacketTransmitting, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacTx",
                       ns3::MakeCallback(&LogMacCommsTracing::MacTx, this));

  ns3::Config::Connect("/ROSDeviceList/*/MacRx",
                       ns3::MakeCallback(&LogMacCommsTracing::MacRx, this));
}

void LogMacCommsTracing::MacRx(std::string path, ROSCommsDevicePtr dev,
                                 ns3ConstPacketPtr pkt) {
    AquaSimHeader header;
    pkt->PeekHeader(header);
    Info("[{}] MAC RX -- ID: {} ; MAC: {} ; Size: {}", path, dev->GetDccommsId(),
         dev->GetMac(), header.GetSize());
}

void LogMacCommsTracing::MacTx(std::string path, ROSCommsDevicePtr dev,
                                 ns3ConstPacketPtr pkt) {
    AquaSimHeader header;
    pkt->PeekHeader(header);
    Info("[{}] MAC TX -- ID: {} ; MAC: {} ; Size: {}", path, dev->GetDccommsId(),
         dev->GetMac(), header.GetSize());
}

void LogMacCommsTracing::DoRun() {
  std::thread work([this]() {
    geometry_msgs::TwistStamped msg;
    msg.twist.linear.x = 0;
    msg.twist.linear.y = 0;
    msg.twist.linear.z = 0;
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = 0;

    double baseVelocity = 0.25;
    double r = 20;
    double itt = 1. / r;
    double range = 5; // meters

    ros::Rate rate(r);

    int counter, its = range / (itt * baseVelocity);

    while (ros::ok()) {
      std::this_thread::sleep_for(2s);
      msg.twist.linear.x = baseVelocity;

      counter = 0;
      while (counter < its && ros::ok()) {
        bluerov0Pub.publish(msg);
        rate.sleep();
        counter += 1;
      }

      std::this_thread::sleep_for(2s);

      msg.twist.linear.x = -baseVelocity;

      counter = 0;
      while (counter < its && ros::ok()) {
        bluerov0Pub.publish(msg);
        rate.sleep();
        counter += 1;
      }
    }
  });
  work.detach();
}

CLASS_LOADER_REGISTER_CLASS(LogMacCommsTracing, NetSimTracing)
} // namespace netsim_tracing_examples
