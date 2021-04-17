#include "online-map-server/online-map-server.h"

#include <landmark-triangulation/landmark-triangulation.h>

namespace online_map_server {
void OnlineMapServer::processMap(const vi_map::MissionId mission_id) {
  CHECK(last_processed_vertex_id_.count(mission_id));

  map_manipulation_->initializeLandmarksFromUnusedFeatureTracksOfMission(
      mission_id, last_processed_vertex_id_[mission_id]);
  landmark_triangulation::retriangulateLandmarksAlongMissionAfterVertex(
      mission_id, last_processed_vertex_id_[mission_id], map_.get());

  anchorMission(mission_id, nullptr);

  last_processed_vertex_id_[mission_id] =
      map_->getLastVertexIdOfMission(mission_id);
}

void OnlineMapServer::anchorMissionEvent(const ros::TimerEvent& /*event*/) {
  for (size_t i = 0; i < map_updated_.size(); i++) {
    if (!map_updated_[i])
      continue;
    processMap(mission_ids_[i]);
    map_updated_[i] = false;
  }
}

void OnlineMapServer::anchorMission(
    const vi_map::MissionId& mission_id,
    const visualization::ViwlsGraphRvizPlotter* /*plotter*/) {
  if (mission_id != mission_ids_[0]) {
    map_anchoring::anchorMission(mission_id, map_.get());
  }
}

}  // namespace online_map_server
