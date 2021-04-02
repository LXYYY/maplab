#ifndef ONLINE_MAP_SERVER_ONLINE_MAP_SERVER_H_
#define ONLINE_MAP_SERVER_ONLINE_MAP_SERVER_H_

#include <boost/filesystem/path.hpp>
#include <map-anchoring/map-anchoring.h>
#include <memory>
#include <mutex>
#include <online-map-builders/keyframed-map-builder.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <vio-common/vio-update-serialization.h>

namespace online_map_server {
class OnlineMapServer {
 public:
  OnlineMapServer(
      const ros::NodeHandle& node_handle,
      const map_sparsification::KeyframingHeuristicsOptions& keyframing_options,
      const vi_map::SensorManager& sensor_manager,
      const std::string& save_map_folder)
      : node_handle_(node_handle),
        map_(aligned_shared<vi_map::VIMap>(save_map_folder)) {
    int client_number = 1;
    node_handle_.param("client_number", client_number, client_number);

    for (int i = 0; i < client_number; i++) {
      keyframed_map_builder_.emplace_back(
          new online_map_builders::KeyframedMapBuilder(
              keyframing_options, sensor_manager, map_.get()));
      vio_update_subscriber_.emplace_back(
          node_handle_.subscribe<std_msgs::String>(
              "vio_update_" + std::to_string(i), 10,
              [this, i](const std_msgs::String::ConstPtr& vio_update_msg) {
                this->VioUpdateCallback(i, vio_update_msg);
              }));
      mission_ids_.emplace_back(keyframed_map_builder_[i]->getMissionId());
    }

    auto const& first_mission_id = keyframed_map_builder_[0]->getMissionId();
    map_anchoring::setMissionBaseframeKnownState(
        first_mission_id, true, map_.get());
  }

  virtual ~OnlineMapServer() = default;

  void VioUpdateCallback(
      int cid, const std_msgs::String::ConstPtr& vio_update_msg) {
    CHECK_NOTNULL(keyframed_map_builder_[cid]);
    vio::proto::VioUpdate vio_update_proto;
    vio_update_proto.ParseFromString(vio_update_msg->data);
    vio::VioUpdate vio_update;
    vio::serialization::deserializeVioUpdate(vio_update_proto, &vio_update);
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      keyframed_map_builder_[cid]->applyUpdateAndKeyframe(
          std::make_shared<const vio::VioUpdate>(vio_update));

      processMap(cid);
    }
  }

  void processMap(int cid);

 private:
  void anchorMission(
      const vi_map::MissionId& mission_id,
      const visualization::ViwlsGraphRvizPlotter* plotter);

  ros::NodeHandle node_handle_;

  std::vector<ros::Subscriber> vio_update_subscriber_;

  std::vector<std::shared_ptr<online_map_builders::KeyframedMapBuilder>>
      keyframed_map_builder_;
  std::vector<vi_map::MissionId> mission_ids_;

  vi_map::VIMap::Ptr map_;

  std::mutex map_mutex_;
};
}  // namespace online_map_server

#endif  // ONLINE_MAP_SERVER_ONLINE_MAP_SERVER_H_
