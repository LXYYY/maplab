#ifndef ROVIOLI_ONLINE_MAP_PUBLISHER_FLOW_H_
#define ROVIOLI_ONLINE_MAP_PUBLISHER_FLOW_H_

#include <map-sparsification/keyframe-pruning.h>
#include <message-flow/message-flow.h>
#include <online-map-builders/keyframed-map-builder.h>
#include <rovioli/map-builder-flow.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#pragma GCC diagnostic pop

#include "rovioli/flow-topics.h"
#include "rovioli/online-map-publisher.h"

namespace rovioli {
class OnlineMapPublisherFlow {
 public:
  OnlineMapPublisherFlow();
  ~OnlineMapPublisherFlow() = default;

  void attachToMessageFlow(message_flow::MessageFlow* flow);

 private:
  ros::NodeHandle node_handle_;

  rovioli::KeyframedMapPublisher map_publisher_;
};

}  // namespace rovioli

#endif  // ROVIOLI_ONLINE_MAP_PUBLISHER_FLOW_H_
