// pose_sync.cpp — sync KUKA model world pose to WAM-V boat via gz-transport
#include <chrono>
#include <string>
#include <thread>
#include <iostream>
#include <functional>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/math/Pose3.hh>

static const std::string kWorldName = "marine_world";
static const std::string kWamvModel = "wamv";
static const std::string kKukaModel = "kuka_kr600";
static const gz::math::Pose3d kDeckOffset(0, 0, 1.0, 0, 0, 0);

int main(int argc, char** argv) {
  (void)argc; (void)argv;

  gz::transport::Node node;
  gz::math::Pose3d wamvPose;
  bool havePose = false;
  int seq = 0;

  // SceneBroadcaster publishes Pose_V with all entity poses
  std::string poseTopic = "/world/" + kWorldName + "/pose/info";
  std::string setPoseSrv = "/world/" + kWorldName + "/set_pose";

  std::function<void(const gz::msgs::Pose_V&)> cb = [&](const gz::msgs::Pose_V &msg) {
    for (int i = 0; i < msg.pose_size(); ++i) {
      const auto &p = msg.pose(i);
      if (p.name() == kWamvModel) {
        wamvPose = gz::msgs::Convert(p);
        havePose = true;
        return;
      }
    }
  };
  node.Subscribe(poseTopic, cb);

  std::cout << "pose_sync: waiting for WAM-V pose on " << poseTopic << "..." << std::endl;

  auto start = std::chrono::steady_clock::now();
  while (!havePose) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) {
      std::cerr << "pose_sync: TIMEOUT — no WAM-V pose received after 10s" << std::endl;
      return 1;
    }
  }

  std::cout << "pose_sync: first WAM-V pose at (" << wamvPose.Pos().X()
            << ", " << wamvPose.Pos().Y() << ", " << wamvPose.Pos().Z() << ")"
            << std::endl;
  std::cout << "pose_sync: syncing " << kKukaModel << " at 50 Hz" << std::endl;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    seq++;

    gz::math::Pose3d kukaWorldPose = wamvPose * kDeckOffset;

    gz::msgs::Pose req = gz::msgs::Convert(kukaWorldPose);
    req.set_name(kKukaModel);

    gz::msgs::Boolean rep;
    bool result = false;
    node.Request(setPoseSrv, req, 500u, rep, result);

    if (seq == 50) {
      std::cout << "pose_sync: 50 calls ok=" << (result ? "y" : "n")
                << " kuka_z=" << kukaWorldPose.Pos().Z() << std::endl;
      seq = 0;
    }
  }

  return 0;
}
