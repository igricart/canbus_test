/*
 * Copyright (c) 2015-2016, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <bitset>
#include <chrono>
#include <cstdint>
#include <thread>

#include "kacanopen/core/core.h"
#include "kacanopen/core/logger.h"
#include "kacanopen/master/device.h"
// This example lists all connected devices and prints their manufacturer device
// name.
int main() {
  // Set the name of your CAN bus. "slcan0" is a common bus name
  // for the first SocketCAN device on a Linux system.
  const std::string busname = "vcan0";
  // Set the baudrate of your CAN bus. Most drivers support the values
  // "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
  const std::string baudrate = "500K";

  const uint8_t node_id = 2;
  std::bitset<265> m_device_alive;
  std::vector<std::unique_ptr<kaco::Device>> m_devices;
  PRINT("List devices");
  kaco::Core core;

  auto start_master = [&]() -> int {
    if (!core.start(busname, baudrate)) {
      ERROR("Starting core failed.");
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  };

  auto device_alive_callback = [&](const uint8_t node_id) {
    if (!m_device_alive.test(node_id)) {
      m_device_alive.set(node_id);
      m_devices.emplace_back(new kaco::Device(core, node_id));
      std::cout << "New node added" << std::endl;
    } else {
      WARN("Device with node ID " << node_id << " already exists. Ignoring...");
    }
  };

  auto setup_node = [&]() {
    core.nmt.register_device_alive_callback(device_alive_callback);
  };

  // connect to specified node
  auto connect_node = [&]() {
    core.nmt.send_nmt_message(node_id, kaco::NMT::Command::reset_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  };

  start_master();
  setup_node();
  connect_node();

  while (!m_devices.size()) {
    std::cout << "No devices found, waiting 1 second" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  std::cout << "Connected to: " << m_devices.size() << std::endl;
}
