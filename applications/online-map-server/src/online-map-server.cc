#include "online-map-server/online-map-server.h"

namespace online_map_server {
void OnlineMapServer::processMap(int cid) {
  anchorMission(mission_ids_[cid], nullptr);
}

void OnlineMapServer::anchorMission(
    const vi_map::MissionId& mission_id,
    const visualization::ViwlsGraphRvizPlotter* plotter) {
  map_anchoring::anchorMission(mission_id, map_.get());
}

}  // namespace online_map_server
