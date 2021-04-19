#ifndef ONLINE_MAP_SERVER_OPTIMIZATION_H_
#define ONLINE_MAP_SERVER_OPTIMIZATION_H_

#include <map-optimization/solver-options.h>
#include <map-optimization/vi-map-relaxation.h>
#include <map-sparsification/keyframe-pruning.h>
#include <map>
#include <vi-map/vi-map.h>

namespace online_map_server {
class Optimization {
 public:
  MAPLAB_POINTER_TYPEDEFS(Optimization);
  explicit Optimization(const vi_map::VIMap::Ptr& map) : map_(map) {}
  virtual ~Optimization() = default;

  bool keyframingAndOptimizeMap(const vi_map::MissionIdList& mission_ids) {
    if (relaxPoseGraph(mission_ids)) {
      if (true /*keyframingMap(mission_ids)*/)  // not keyfrming. I think maybe
                                                // need to find loop closure
                                                // again after keyframing, which
                                                // costs some time
        return true; /*optimizeMap();*/  // not optimize, because relaxation
                                         // optimized the pose graph and
                                         // generated a globally consistent map,
                                         // maybe good enough to use
      else
        return false;
    } else {
      return false;
    }
  }

 private:
  bool relaxPoseGraph(const vi_map::MissionIdList& mission_ids) {
    map_optimization::VIMapRelaxation relaxation(nullptr, true);

    //  ceres::Solver::Options solver_options =
    //      map_optimization::initSolverOptionsFromFlags();

    //  vi_map::MissionIdSet mission_id_set(mission_ids.begin(),
    //  mission_ids.end()); return relaxation.solveRelaxation(
    //      solver_options, mission_id_set, map_.get());
    //
    return relaxation.findLoopClosuresAndSolveRelaxation(
        mission_ids, map_.get());
  }

  bool keyframingMap(const vi_map::MissionIdList& mission_ids) {
    for (auto const& mission_id : mission_ids) {
      const size_t num_initial_vertices =
          map_->numVerticesInMission(mission_id);

      pose_graph::VertexId root_vertex_id;
      if (!last_processed_vertex_id_.count(mission_id))
        root_vertex_id = map_->getMission(mission_id).getRootVertexId();
      else
        root_vertex_id = last_processed_vertex_id_[mission_id];

      CHECK(root_vertex_id.isValid());
      pose_graph::VertexId last_vertex_id =
          map_->getLastVertexIdOfMission(mission_id);

      pose_graph::VertexIdList keyframe_ids;
      map_sparsification::KeyframingHeuristicsOptions keyframing_options =
          map_sparsification::KeyframingHeuristicsOptions::
              initializeFromGFlags();
      map_sparsification::selectKeyframesBasedOnHeuristics(
          *map_, root_vertex_id, last_vertex_id, keyframing_options,
          &keyframe_ids);
      if (keyframe_ids.empty())
        return false;
      else
        VLOG(1) << "Selected " << keyframe_ids.size() << " keyframes of "
                << num_initial_vertices << " vertices.";

      // Remove non-keyframe vertices.
      const size_t num_removed_keyframes =
          map_sparsification::removeVerticesBetweenKeyframes(
              keyframe_ids, map_.get());
      VLOG(1) << "Removed " << num_removed_keyframes << " vertices of "
              << num_initial_vertices << " vertices.";
      if (!last_processed_vertex_id_.count(mission_id))
        last_processed_vertex_id_.emplace(mission_id, last_vertex_id);
    }

    return true;
  }

  bool optimizeMap() {
    return true;
  }

  vi_map::VIMap::Ptr map_;
  std::map<vi_map::MissionId, pose_graph::VertexId> last_processed_vertex_id_;
};
}  // namespace online_map_server

#endif  // ONLINE_MAP_SERVER_OPTIMIZATION_H_
