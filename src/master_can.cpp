#include <ros/ros.h>

#include <bitset>
#include <chrono>
#include <cstdint>
#include <thread>

#include "kacanopen/core/canopen_error.h"
#include "kacanopen/core/core.h"
#include "kacanopen/core/logger.h"
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

 protected:
  /* data */
  std::string busname_;
  std::string baudrate_;
  int node_id_;
  std::vector<std::unique_ptr<kaco::Device>> m_devices_;
  kaco::Core core_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::bitset<265> m_device_alive_;
};

MasterCANInterface::MasterCANInterface(ros::NodeHandle& nh)
    : nh_(nh), pnh_("~") {
  pnh_.param<int>("node_id", node_id_, 2);
  pnh_.param<std::string>("busname", busname_, "can0");
  pnh_.param<std::string>("baudrate", baudrate_, "1M");
};

MasterCANInterface::~MasterCANInterface(){};

/**
 * @brief Master CAN Implementation
 *
 */
class MasterCAN : public MasterCANInterface {
 private:
 public:
  MasterCAN(ros::NodeHandle& nh);
  ~MasterCAN();

  bool start() final;
  void setup_node() final;
  void connect_node() final;
  size_t get_devices_vec_size();
  void get_device_info();
};

MasterCAN::MasterCAN(ros::NodeHandle& nh) : MasterCANInterface(nh) {}

MasterCAN::~MasterCAN() {}

bool MasterCAN::start() {
  if (!core_.start(busname_, baudrate_)) {
    ERROR("Starting core failed.");
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

void MasterCAN::setup_node() {
  auto device_alive_callback = [&](const int node_id) {
    if (!m_device_alive_.test(node_id)) {
      m_device_alive_.set(node_id);
      m_devices_.emplace_back(new kaco::Device(core_, node_id));
      std::cout << "New node added" << std::endl;
    } else {
      WARN("Device with node ID " << node_id << " already exists. Ignoring...");
    }
  };
  core_.nmt.register_device_alive_callback(device_alive_callback);
}

void MasterCAN::connect_node() {
  core_.nmt.send_nmt_message(node_id_, kaco::NMT::Command::reset_node);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
};

size_t MasterCAN::get_devices_vec_size() { return m_devices_.size(); }

void MasterCAN::get_device_info() {
  std::cout << "Load EDS file." << std::endl;
  m_devices_[0]->load_dictionary_from_eds(
      "/home/igricart/Documents/ps/tmp/canbus/src/canbus_test/resources/"
      "Assisted Docking PC - Master.eds");
  std::cout << "Starting device with ID " << (unsigned)node_id_ << "..."
            << std::endl;
  m_devices_[0]->start();
  std::cout << "Loading object dictionary from the library. This can be either "
               "a generic CiA-301"
            << " dictionary, a CiA-profile specific dictionary, or a "
               "manufacturer./device-specific dictionary."
            << std::endl;
  std::cout << "The following should work for all CANopen-compliant devices."
            << std::endl;
  std::cout << "CiA-profile = " << m_devices_[0]->get_device_profile_number()
            << std::endl;
  std::cout << "Vendor-ID = "
            << m_devices_[0]->get_entry("Identity object/Vendor-ID")
            << std::endl;
  std::cout << "The following works for most CANopen-compliant devices "
               "(however, the entries are not mandatory)."
            << std::endl;
  m_devices_[0]->print_dictionary();
}

/**
 * @brief Main Process
 *
 * @return int
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "master_can");
  ros::NodeHandle nh("");
  MasterCAN master(nh);

  master.start();
  master.setup_node();
  master.connect_node();
  master.get_device_info();

  while (!master.get_devices_vec_size()) {
    std::cout << "No devices found, waiting 1 second" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  std::cout << "Connected to: " << master.get_devices_vec_size() << std::endl;

  return 0;
}
