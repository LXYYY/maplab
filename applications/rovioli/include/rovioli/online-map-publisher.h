#ifndef ROVIOLI_ONLINE_MAP_PUBLISHER_H_
#define ROVIOLI_ONLINE_MAP_PUBLISHER_H_

#include <map-sparsification/keyframe-pruning.h>
#include <mutex>
#include <ros/ros.h>
#include <rovioli/vi-map-with-mutex.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <vi-map/vi-map.h>
#include <vio-common/map-update.h>

namespace rovioli {
class KeyframedMapPublisher {
 public:
  KeyframedMapPublisher() {
    keyframe_publisher_ =
        node_handle_.advertise<std_msgs::String>("keyframes", 1, true);
    edge_publisher_ =
        node_handle_.advertise<std_msgs::String>("edges", 1, true);
    landmark_publisher_ =
        node_handle_.advertise<std_msgs::String>("landmarks", 1, true);
  }
  ~KeyframedMapPublisher() = default;

  void apply(const VIMapWithMutex::ConstPtr& /*map_with_mutex*/) {
    {
      std::lock_guard<std::mutex> lock(map_with_mutex_->mutex);
      pose_graph::VertexId start_keyframe_id;
      pose_graph::VertexId end_vertex_id;
      map_sparsification::KeyframingHeuristicsOptions keyframing_options;
      keyframing_options.initializeFromGFlags();
      std::vector<pose_graph::VertexId> keyframe_ids;
      map_sparsification::selectKeyframesBasedOnHeuristics(
          map_with_mutex_->vi_map, start_keyframe_id, end_vertex_id,
          keyframing_options, &keyframe_ids);

      for (auto const& keyframe_id : keyframe_ids) {
        auto& keyframe_vertex = map_with_mutex_->vi_map.getVertex(keyframe_id);
        vi_map::proto::ViwlsVertex vertex_proto;
        keyframe_vertex.serialize(&vertex_proto);
        std::string vertex_msg;
        vertex_proto.SerializeToString(&vertex_msg);
        keyframe_publisher_.publish(vertex_msg);

        pose_graph::EdgeIdSet edge_id_set;
        keyframe_vertex.getAllEdges(&edge_id_set);
        for (auto const& edge_id : edge_id_set) {
          vi_map::Edge edge =
              map_with_mutex_->vi_map.getEdgeAs<vi_map::Edge>(edge_id);
          // TODO(mikexyl): add check if edge between keyframes
          vi_map::proto::Edge edge_proto;
          edge.serialize(&edge_proto);
          std::string edge_msg;
          edge_proto.SerializeToString(&edge_msg);
          edge_publisher_.publish(edge_msg);
        }

        // TODO(mikexyl): looks like keyframe proto has landmarks in it
        // auto& landmarks=keyframe_vertex.getLandmarks();
        // vi_map::proto::LandmarkStore landmarks_proto;
        // landmarks.serialize(&landmarks_proto);
        // std::string landmarks_msg;
        // landmarks_proto.SerializeToString(&landmarks_msg);
        // landmark_publisher_.publish(landmarks_msg);
      }
    }
  }

 private:
  ros::NodeHandle node_handle_;

  ros::Publisher keyframe_publisher_;
  ros::Publisher edge_publisher_;
  ros::Publisher landmark_publisher_;

  pose_graph::EdgeIdSet published_edges_;
  std::vector<pose_graph::VertexId> published_keyframes_;
  pose_graph::VertexId last_keyframe_;

  VIMapWithMutex::Ptr map_with_mutex_;
};

}  // namespace rovioli

#endif  // ROVIOLI_ONLINE_MAP_PUBLISHER_H_
