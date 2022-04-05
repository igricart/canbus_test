#pragma once

#include <ros/ros.h>

#include "kacanopen/master/device.h"

/**
 * @brief Master CAN Interface
 *
 */
class MasterCANInterface {
 private:
 public:
  MasterCANInterface(ros::NodeHandle& nh);
  virtual ~MasterCANInterface();

  bool virtual start() = 0;
  void virtual setup_node() = 0;
  void virtual connect_node() = 0;
  bool connected() { return connected_; };

 protected:
  // Ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Params
  std::string busname_;
  std::string baudrate_;
  int sample_rate_;  // Loop cycle time (ms)
  int node_id_;      // Node ID from PLC
  int heartbeat_interval_;

  // Kacanopen
  std::unique_ptr<kaco::Device> device_;
  std::unordered_map<uint16_t, std::string> pdo_input_map, pdo_output_map;
  kaco::Core core_;

  // States
  bool connected_;
};

MasterCANInterface::MasterCANInterface(ros::NodeHandle& nh)
    : nh_(nh), pnh_("~"), connected_(false) {
  pnh_.param<int>("node_id", node_id_, 2);
  pnh_.param<std::string>("busname", busname_, "can0");
  pnh_.param<std::string>("baudrate", baudrate_, "1M");
  pnh_.param<int>("sample_rate", sample_rate_, 100);
  pnh_.param<int>("heartbeat_interval", heartbeat_interval_, 250);
};

MasterCANInterface::~MasterCANInterface(){};
