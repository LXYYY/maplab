#include "online-map-server/online-map-server.h"

#include <landmark-triangulation/landmark-triangulation.h>
#include <map-sparsification-plugin/keyframe-pruning.h>

namespace online_map_server {
void OnlineMapServer::processMap(const vi_map::MissionId& mission_id) {
  CHECK(last_processed_vertex_id_.count(mission_id));

  initializeLandMarks(mission_id);
  anchorMission(mission_id, nullptr);

  // optimizeMap();

  last_processed_vertex_id_[mission_id] =
      map_->getLastVertexIdOfMission(mission_id);
}

void OnlineMapServer::initializeLandMarks(const vi_map::MissionId& mission_id) {
  map_manipulation_->initializeLandmarksFromUnusedFeatureTracksOfMission(
      mission_id, last_processed_vertex_id_[mission_id]);
  landmark_triangulation::retriangulateLandmarksAlongMissionAfterVertex(
      mission_id, last_processed_vertex_id_[mission_id], map_.get());
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
  CHECK(last_processed_vertex_id_.count(mission_id));
  pose_graph::VertexId last_vertex_id = last_processed_vertex_id_[mission_id];
  CHECK(map_->hasVertex(last_vertex_id));
  pose_graph::VertexId next_vertex_id;
  pose_graph::VertexIdList new_vertex_ids;
  if (map_->getMissionBaseFrameForMission(mission_id).is_T_G_M_known()) {
    if (map_->getNextVertex(
            last_vertex_id, map_->getGraphTraversalEdgeType(mission_id),
            &next_vertex_id))
      map_->getAllVertexIdsInMissionAlongGraph(
          mission_id, next_vertex_id, &new_vertex_ids);
    else
      return;
  } else {
    map_->getAllVertexIdsInMissionAlongGraph(mission_id, &new_vertex_ids);
  }

  // Try to anchor mission if its not the first mission
  if (mission_id != mission_ids_[0]) {
    map_anchoring::ProbeResult loop_result;
    vi_map::LoopClosureConstraintVector inlier_constraints;
    loop_detector_.detectLoopClosuresVerticesToDatabase(
        new_vertex_ids, true, true, &loop_result.num_vertex_candidate_links,
        &loop_result.average_landmark_match_inlier_ratio, map_.get(),
        &loop_result.T_G_M, &inlier_constraints);
    LOG(INFO) << "Found " << inlier_constraints.size() << " loops";
    if (loop_result.wasSuccessful()) {
      LOG(INFO) << "Anchoring agent " << mid_cid_map_[mission_id] << " mission "
                << mission_id;
      VLOG(1) << "With T_G_M:" << std::endl << loop_result.T_G_M;

      // Prealign
      map_->getMissionBaseFrameForMission(mission_id)
          .set_T_G_M(loop_result.T_G_M);
      map_->getMissionBaseFrameForMission(mission_id).set_is_T_G_M_known(true);
    }
  }

  // Add all anchored mission to database to find loop among all vertices
  if (map_->getMissionBaseFrameForMission(mission_id).is_T_G_M_known())
    loop_detector_.addVerticesToDatabase(new_vertex_ids, *map_);
}

}  // namespace online_map_server
