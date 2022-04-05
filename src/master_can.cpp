#include <ros/package.h>
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
  int sample_rate_;  // Loop cycle time (ms)
  int node_id_;      // Node ID from PLC
  std::unique_ptr<kaco::Device> device_;
  std::unordered_map<uint16_t, std::string> pdo_input_map, pdo_output_map;
  kaco::Core core_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::bitset<265> m_device_alive_;
  int heartbeat_interval_;
};

MasterCANInterface::MasterCANInterface(ros::NodeHandle& nh)
    : nh_(nh), pnh_("~") {
  pnh_.param<int>("node_id", node_id_, 2);
  pnh_.param<std::string>("busname", busname_, "can0");
  pnh_.param<std::string>("baudrate", baudrate_, "1M");
  pnh_.param<int>("sample_rate", sample_rate_, 100);
  pnh_.param<int>("heartbeat_interval", heartbeat_interval_, 250);
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
  void get_device_info();
  bool create_pdo_mapping();
  void read();
  void send(u_int8_t i);
  void communicate();
  u_int8_t value_;
  std::mutex device_mutex_;
};

MasterCAN::MasterCAN(ros::NodeHandle& nh) : MasterCANInterface(nh) {}

MasterCAN::~MasterCAN() {}

bool MasterCAN::start() {
  std::cout << "Start Core" << std::endl;
  if (!core_.start(busname_, baudrate_)) {
    ERROR("Starting core failed.");
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

void MasterCAN::setup_node() {
  std::cout << "Setup Node" << std::endl;
  auto device_alive_callback = [&](const int node_id) {
    std::lock_guard<std::mutex> lock(device_mutex_);
    std::cout << "Device Alive Callback" << std::endl;
    if (!m_device_alive_.test(node_id)) {
      m_device_alive_.set(node_id);
      device_ = std::make_unique<kaco::Device>(core_, node_id);
      std::cout << "New node added" << std::endl;
    } else {
      WARN("Device with node ID " << node_id << " already exists. Ignoring...");
    }

    device_->set_entry(0x1017, 0x0, heartbeat_interval_,
                       kaco::WriteAccessMethod::sdo);
  };

  auto device_dead_callback = [&](const uint8_t node_id) {
    std::lock_guard<std::mutex> lock(device_mutex_);
    std::cout << "Device Dead Callback" << std::endl;
    // Check whether we already have this node_id
  };

  core_.nmt.register_device_alive_callback(device_alive_callback);
  core_.nmt.register_device_dead_callback(device_dead_callback);
}

void MasterCAN::connect_node() {
  core_.nmt.send_nmt_message(node_id_, kaco::NMT::Command::reset_node);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
};

void MasterCAN::get_device_info() {
  if (!device_) {
    return;
  }
  std::cout << "Load EDS file." << std::endl;
  device_->load_dictionary_from_eds(
      ros::package::getPath("canbus_test") +
      "/resources/Assisted Docking PC - Master.eds");
  std::cout << "Starting device with ID " << (unsigned)node_id_ << "..."
            << std::endl;
  device_->start();
  std::cout << "Loading object dictionary from the library. This can be either "
               "a generic CiA-301"
            << " dictionary, a CiA-profile specific dictionary, or a "
               "manufacturer./device-specific dictionary."
            << std::endl;
  std::cout << "The following should work for all CANopen-compliant devices."
            << std::endl;
  std::cout << "CiA-profile = " << device_->get_device_profile_number()
            << std::endl;
  std::cout << "Vendor-ID = " << device_->get_entry("Identity object/Vendor-ID")
            << std::endl;
  std::cout << "The following works for most CANopen-compliant devices "
               "(however, the entries are not mandatory)."
            << std::endl;
  device_->print_dictionary();
}

bool MasterCAN::create_pdo_mapping() {
  if (!device_) {
    return false;
  }
  try {
    std::cout << "Create PDO Mapping" << std::endl;
    // Receiver Ports
    device_->add_receive_pdo_mapping(0x1A1,
                                     "Digital_Inputs1_1/Digital_Inputs1_1", 0);

    device_->add_receive_pdo_mapping(0x2A1,
                                     "Digital_Inputs2_1/Digital_Inputs2_1", 0);

    device_->add_receive_pdo_mapping(0x4A2,
                                     "Digital_Inputs5_1/Digital_Inputs5_1", 0);

    // Transmission Ports
    device_->add_transmit_pdo_mapping(
        0x200 + node_id_, {{"digital_outputs1_1/digital_outputs1_1", 0}},
        kaco::TransmissionType::ON_CHANGE, std::chrono::milliseconds(20));

    // Sleep required to setup the communication
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return false;
  }
}

void MasterCAN::read() {
  if (!device_) {
    return;
  }
  try {
    auto value_1 =
        device_->get_entry("Digital_Inputs1_1/Digital_Inputs1_1",
                           kaco::ReadAccessMethod::pdo_request_and_wait);
    std::cout << "value_1 = " << value_1 << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  }

  try {
    auto value_2 =
        device_->get_entry("Digital_Inputs2_1/Digital_Inputs2_1",
                           kaco::ReadAccessMethod::pdo_request_and_wait);
    std::cout << "value_2 = " << value_2 << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  }

  try {
    auto value_5 =
        device_->get_entry("Digital_Inputs5_1/Digital_Inputs5_1",
                           kaco::ReadAccessMethod::pdo_request_and_wait);
    std::cout << "value_5 = " << value_5 << std::endl;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  }
}

void MasterCAN::send(u_int8_t i) {
  if (!device_) {
    return;
  }
  std::cout << "Sending: " << i << std::endl;
  device_->set_entry("digital_outputs1_1/digital_outputs1_1",
                     static_cast<u_int8_t>(i), kaco::WriteAccessMethod::pdo);
}

void MasterCAN::communicate() {
  std::lock_guard<std::mutex> lock(device_mutex_);
  bool running = true;
  u_int8_t i = 0;
  while (running) {
    this->send(i);
    this->read();

    std::this_thread::sleep_for(std::chrono::milliseconds(this->sample_rate_));
    ++i;
  }
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

  // while (true) {
  //   std::cout << "No devices found, waiting 1 second" << std::endl;
  //   std::this_thread::sleep_for(std::chrono::seconds(1));
  // }

  while (!master.create_pdo_mapping()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  master.create_pdo_mapping();
  master.communicate();
  return 0;
}
