#ifndef ONLINE_MAP_SERVER_ONLINE_MAP_SERVER_H_
#define ONLINE_MAP_SERVER_ONLINE_MAP_SERVER_H_

#include <boost/filesystem/path.hpp>
#include <map-anchoring/map-anchoring.h>
#include <map>
#include <memory>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <mutex>
#include <nav_msgs/Path.h>
#include <online-map-builders/keyframed-map-builder.h>
#include <online-map-builders/stream-map-builder.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <vi-map/sensor-utils.h>
#include <vio-common/map-update.h>
#include <vio-common/vio-update-serialization.h>
#include <voxblox/utils/timing.h>

#include "online-map-server/optimization.h"

namespace online_map_server {
class OnlineMapServer {
 public:
  OnlineMapServer(
      const ros::NodeHandle& node_handle,
      const map_sparsification::
          KeyframingHeuristicsOptions& /*keyframing_options*/,
      const vi_map::SensorManager& sensor_manager,
      const std::string& save_map_folder)
      : node_handle_(node_handle),
        map_(aligned_shared<vi_map::VIMap>(save_map_folder)) {
    int client_number = 1;
    node_handle_.param("client_number", client_number, client_number);

    for (int i = 0; i < client_number; i++) {
      keyframed_map_builder_.emplace_back(
          new online_map_builders::StreamMapBuilder(
              sensor_manager, map_.get()));
      vio_update_subscriber_.emplace_back(
          node_handle_.subscribe<std_msgs::String>(
              "vio_update_" + std::to_string(i), 10,
              [this, i](const std_msgs::String::ConstPtr& vio_update_msg) {
                this->vioUpdateCallback(i, vio_update_msg);
              }));
      auto const& mission_id = keyframed_map_builder_[i]->getMissionId();
      mission_ids_.emplace_back(mission_id);
      mid_cid_map_.emplace(mission_id, i);
      map_updated_.emplace_back(false);
      mission_num_lc_links_.emplace(mission_id, 0);
    }

    map_anchoring::setMissionBaseframeKnownState(
        mission_ids_[0], true, map_.get());
    ncamera_ = vi_map::getSelectedNCamera(sensor_manager);

    float anchor_mission_every_n_sec = 1.0;
    node_handle_.param(
        "anchor_mission_every_n_sec", anchor_mission_every_n_sec,
        anchor_mission_every_n_sec);
    anchor_mission_timer_ = node_handle_.createTimer(
        ros::Duration(anchor_mission_every_n_sec),
        &OnlineMapServer::anchorMissionEvent, this);

    float publish_path_every_n_sec = 5.0;
    if (publish_path_every_n_sec > 0) {
      node_handle_.param(
          "publish_path_every_n_sec", publish_path_every_n_sec,
          publish_path_every_n_sec);
      //  optimize_publish_timer_ = node_handle_.createTimer(
      //      ros::Duration(publish_path_every_n_sec),
      //      &OnlineMapServer::optimizeAndPublishEvent, this);

      for (int i = 0; i < client_number; i++) {
        path_pub_.emplace_back(node_handle_.advertise<nav_msgs::Path>(
            "agent_path_" + std::to_string(i), 10));
      }
    }

    map_manipulation_.reset(new vi_map_helpers::VIMapManipulation(map_.get()));

    optimization_.reset(new Optimization(map_));
  }

  virtual ~OnlineMapServer() = default;

  void vioUpdateCallback(
      int cid, const std_msgs::String::ConstPtr& vio_update_msg) {
    CHECK_NOTNULL(keyframed_map_builder_[cid]);
    vio::proto::VioUpdate vio_update_proto;
    voxblox::timing::Timer timer("string_parsing");
    vio_update_proto.ParseFromString(vio_update_msg->data);
    vio::VioUpdate vio_update;
    vio::serialization::deserializeVioUpdate(
        vio_update_proto, ncamera_, &vio_update);
    timer.Stop();
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      keyframed_map_builder_[cid]->apply(
          vio::MapUpdate::fromVioUpdate(vio_update));
      auto const& mission_id = mission_ids_[cid];
      if (!last_processed_vertex_id_.count(mission_id)) {
        auto const& root_vertex_id =
            map_->getMission(mission_id).getRootVertexId();
        last_processed_vertex_id_.emplace(mission_id, root_vertex_id);
      }

      map_updated_[cid] = true;
    }
  }

 private:
  void anchorMissionEvent(const ros::TimerEvent& /*event*/);
  void optimizeAndPublishEvent(const ros::TimerEvent& /*event*/) {
    optimizeMap();
  }
  void optimizeMap() {
    vi_map::MissionIdList missions_ids_known_T_G_M;
    for (auto const& mission_id : mission_ids_)
      if (map_->getMissionBaseFrameForMission(mission_id).is_T_G_M_known())
        missions_ids_known_T_G_M.emplace_back(mission_id);

    // TODO(mikexyl): for some reason, relaxation can't find loop closure
    if (missions_ids_known_T_G_M.size() > 1)
      // optimization_->keyframingAndOptimizeMap(missions_ids_known_T_G_M);
      optimization_->relaxPoseGraph(mission_ids_);
  }

  void publishPath() {
    for (size_t cid = 0; cid < mission_ids_.size(); cid++) {
      auto const& mission_id = mission_ids_[cid];
      if (map_->getMissionBaseFrameForMission(mission_id).is_T_G_M_known()) {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map_" + std::to_string(cid);
        auto const& root_vertex_id =
            map_->getMission(mission_id).getRootVertexId();
        pose_graph::VertexIdList vertices;
        map_->getAllVertexIdsInMissionAlongGraph(
            mission_id, root_vertex_id, &vertices);
        for (auto const& vertex_id : vertices) {
          auto const& vertex = map_->getVertex(vertex_id);
          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.header.frame_id = "map_" + std::to_string(cid);
          ros::Time stamp;
          stamp.fromNSec(vertex.getVisualFrame(0).getTimestampNanoseconds());
          pose_stamped.header.stamp = stamp;
          tf::poseKindrToMsg(vertex.get_T_M_I(), &pose_stamped.pose);
          path_msg.poses.emplace_back(pose_stamped);
        }
        path_pub_[cid].publish(path_msg);
      }
    }
  }

  void publishTfBetweenMission() {
    for (auto const& mission_id : mission_ids_) {
      auto const& T_G_M =
          map_->getMissionBaseFrameForMission(mission_id).get_T_G_M();
      tf::Transform T_G_M_tf;
      tf::transformKindrToTF(T_G_M, &T_G_M_tf);
      tf_br_.sendTransform(tf::StampedTransform(
          T_G_M_tf, ros::Time::now(), "world",
          std::to_string(mid_cid_map_[mission_id])));
    }
  }

  void processMap(const vi_map::MissionId& mission_id);
  void initializeLandMarks(const vi_map::MissionId& mission_id);
  void anchorMission(
      const vi_map::MissionId& mission_id,
      const visualization::ViwlsGraphRvizPlotter* plotter);

  ros::NodeHandle node_handle_;
  ros::Timer anchor_mission_timer_;
  ros::Timer optimize_publish_timer_;
  std::vector<ros::Publisher> path_pub_;
  tf::TransformBroadcaster tf_br_;

  std::vector<ros::Subscriber> vio_update_subscriber_;

  std::vector<std::shared_ptr<online_map_builders::StreamMapBuilder>>
      keyframed_map_builder_;
  std::vector<vi_map::MissionId> mission_ids_;
  std::map<vi_map::MissionId, int> mid_cid_map_;
  std::map<vi_map::MissionId, int> mission_num_lc_links_;
  std::map<vi_map::MissionId, pose_graph::VertexId> last_processed_vertex_id_;
  loop_detector_node::LoopDetectorNode loop_detector_;

  aslam::NCamera::Ptr ncamera_;

  vi_map::VIMap::Ptr map_;
  std::shared_ptr<vi_map_helpers::VIMapManipulation> map_manipulation_;
  std::vector<bool> map_updated_;
  std::mutex map_mutex_;

  Optimization::Ptr optimization_;
};
}  // namespace online_map_server

#endif  // ONLINE_MAP_SERVER_ONLINE_MAP_SERVER_H_
