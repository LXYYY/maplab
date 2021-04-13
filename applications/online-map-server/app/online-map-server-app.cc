#include <chrono>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <maplab-ros-common/gflags-interface.h>
#include <ros/ros.h>
#include <vi-map/sensor-utils.h>

#include "online-map-server/online-map-server.h"

DEFINE_string(sensor_calibration_file, "", "Path to sensor calibration yaml.");

DEFINE_string(
    save_map_folder, "", "Save map to folder; if empty nothing is saved.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "online_map_server");
  ros::NodeHandle nh, nh_private("~");
  ros_common::parseGflagsFromRosParams(argv[0], nh_private);

  map_sparsification::KeyframingHeuristicsOptions keyframing_options =
      map_sparsification::KeyframingHeuristicsOptions::initializeFromGFlags();

  // Load sensors.
  CHECK(!FLAGS_sensor_calibration_file.empty());
  vi_map::SensorManager sensor_manager;
  if (!sensor_manager.deserializeFromFile(FLAGS_sensor_calibration_file)) {
    LOG(FATAL) << "[MAP_SERVER] Failed to read the sensor calibration from '"
               << FLAGS_sensor_calibration_file << "'!";
  }

  CHECK(vi_map::getSelectedImu(sensor_manager))
      << "[MAP_SERVER] The sensor calibration does not contain an IMU!";

  aslam::NCamera::Ptr mapping_ncamera =
      vi_map::getSelectedNCamera(sensor_manager);
  CHECK(mapping_ncamera)
      << "[MAP_SERVER] The sensor calibration does not contain a NCamera!";

  online_map_server::OnlineMapServer online_map_server(
      nh_private, keyframing_options, sensor_manager, FLAGS_save_map_folder);

  ros::AsyncSpinner ros_spinner(common::getNumHardwareThreads());
  ros_spinner.start();

  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
