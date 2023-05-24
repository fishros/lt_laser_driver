#ifndef LTME_NODE_H
#define LTME_NODE_H

#include "ltme_interfaces/srv/query_serial.hpp"
#include "ltme_interfaces/srv/query_firmware_version.hpp"
#include "ltme_interfaces/srv/query_hardware_version.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "ldcp/device.h"

#include <mutex>
#include <atomic>

class LidarDriver
{
public:
  const static std::string DEFAULT_FRAME_ID;
  const static bool DEFAULT_INVERT_FRAME;
  const static int DEFAULT_SCAN_FREQUENCY;
  const static double ANGLE_MIN_LIMIT;
  const static double ANGLE_MAX_LIMIT;
  const static double DEFAULT_ANGLE_EXCLUDED_MIN;
  const static double DEFAULT_ANGLE_EXCLUDED_MAX;
  const static double RANGE_MIN_LIMIT;
  const static double RANGE_MAX_LIMIT_02A;
  const static double RANGE_MAX_LIMIT_R1;
  const static double RANGE_MAX_LIMIT_R2;
  const static double RANGE_MAX_LIMIT_I1;
  const static double RANGE_MAX_LIMIT_I2;
  const static int DEFAULT_AVERAGE_FACTOR;
  const static int DEFAULT_SHADOW_FILTER_STRENGTH;

public:
  LidarDriver();
  void run();

private:
  bool querySerialService(const std::shared_ptr<ltme_interfaces::srv::QuerySerial::Request> request,
                          std::shared_ptr<ltme_interfaces::srv::QuerySerial::Response> response);
  bool queryFirmwareVersion(const std::shared_ptr<ltme_interfaces::srv::QueryFirmwareVersion::Request> request,
                            std::shared_ptr<ltme_interfaces::srv::QueryFirmwareVersion::Response> response);
  bool queryHardwareVersion(const std::shared_ptr<ltme_interfaces::srv::QueryHardwareVersion::Request> request,
                            std::shared_ptr<ltme_interfaces::srv::QueryHardwareVersion::Response> response);
  bool requestHibernationService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                 std::shared_ptr<std_srvs::srv::Empty::Response> response);
  bool requestWakeUpService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response);
  bool quitDriverService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                         std::shared_ptr<std_srvs::srv::Empty::Response> response);

private:
  std::shared_ptr<rclcpp::Node> node_;

  std::string device_model_;
  std::string device_address_;
  std::string frame_id_;
  bool invert_frame_;
  double angle_min_;
  double angle_max_;
  double angle_excluded_min_;
  double angle_excluded_max_;
  double range_min_;
  double range_max_;
  int average_factor_;
  int shadow_filter_strength_;

  std::unique_ptr<ldcp_sdk::Device> device_;
  std::mutex mutex_;

  std::atomic_bool hibernation_requested_;
  std::atomic_bool quit_driver_;
};

#endif
