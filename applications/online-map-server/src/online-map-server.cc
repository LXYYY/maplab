#include "online-map-server/online-map-server.h"

#include <landmark-triangulation/landmark-triangulation.h>

namespace online_map_server {
void OnlineMapServer::processMap(int cid) {
  anchorMission(mission_ids_[cid], nullptr);
}

void OnlineMapServer::anchorMission(
    const vi_map::MissionId& mission_id,
    const visualization::ViwlsGraphRvizPlotter* /*plotter*/) {
  vi_map::MissionIdList mission_ids{mission_id};
  landmark_triangulation::retriangulateLandmarks(mission_ids, map_.get());
  if (!map_->getMissionBaseFrameForMission(mission_id).is_T_G_M_known())
    map_anchoring::anchorMission(mission_id, map_.get());
}

}  // namespace online_map_server
