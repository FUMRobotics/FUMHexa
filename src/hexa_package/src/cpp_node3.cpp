#include <ros/package.h>
#include <signal.h>
#include <boost/filesystem.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <ctime>
#include <ratio>
#include "kacanopen/core/canopen_error.h"
#include "kacanopen/core/core.h"
#include "kacanopen/core/logger.h"
#include "kacanopen/master/device.h"
#include "kacanopen/master/master.h"
#include "kacanopen/tools/device_rpdo.h"
#include "kacanopen/tools/device_tpdo.h"
#include "kacanopen/core/logger.h"
#include "kacanopen/ros_bridge/bridge.h"
#include "kacanopen/ros_bridge/entry_publisher.h"
#include "kacanopen/ros_bridge/entry_subscriber.h"
#include "kacanopen/ros_bridge/joint_state_publisher.h"
#include "kacanopen/ros_bridge/joint_state_subscriber.h"

#include <chrono>
#include <memory>
#include <thread>



static volatile int keepRunning = 1;

void intHandler(int dummy) {
  (void)dummy;
  keepRunning = 0;
}

bool printDeviceInfo(std::shared_ptr<kaco::Device> device) {
  try {
    std::string display_device_type;
    auto device_type =
        device->get_entry(0x1000, 0x0, kaco::ReadAccessMethod::sdo);
    if (131474 == static_cast<uint32_t>(device_type))
      display_device_type = "DS402";  // 131474 is the type number for DS-402;
    auto device_name =
        device->get_entry(0x1008, 0x0, kaco::ReadAccessMethod::sdo);
    auto vendor_id =
        device->get_entry(0x1018, 0x01, kaco::ReadAccessMethod::sdo);
    auto product_id =
        device->get_entry(0x1018, 0x02, kaco::ReadAccessMethod::sdo);
    auto revision =
        device->get_entry(0x1018, 0x03, kaco::ReadAccessMethod::sdo);

    auto serial_no =
        device->get_entry(0x1018, 0x04, kaco::ReadAccessMethod::sdo);

    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "* Device Name found as '" << device_name << "'" << std::endl;
    std::cout << "* Device Type found as CiA-" << display_device_type << "'"
              << std::endl;
    std::cout << "* Vendor ID=" << vendor_id << std::endl;
    std::cout << "* Product ID=" << product_id << std::endl;
    std::cout << "* Serial Number=" << serial_no << std::endl;
    std::cout << "* Revision Number=" << revision << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "*************************************************************"
              << std::endl;
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    return true;
  } catch (...) {
    std::cout << "Exception occured while acquiring device information.!"
              << std::endl;
    return false;
  }
}

void initializeDevice(std::shared_ptr<kaco::Device> device,
                      uint16_t heartbeat_interval, uint8_t node_id) {
  // set the our desired heartbeat_interval time
  device->set_entry(0x1017, 0x0, heartbeat_interval,
                    kaco::WriteAccessMethod::sdo);
                    

  // Mater side Periodic Tranmit pdo1 value initialization
  device->set_entry("current_mode_setting_value", static_cast<int16_t>(0x0), kaco::WriteAccessMethod::sdo);
  
  device->set_entry("controlword", static_cast<uint16_t>(0x0006),
                    kaco::WriteAccessMethod::sdo);

  // Master side tpdo1 mapping
  device->add_transmit_pdo_mapping(
      0x200 + node_id, {{"current_mode_setting_value", 0}, {"controlword", 2}},
      kaco::TransmissionType::PERIODIC, std::chrono::milliseconds(5));
  
  // Master side RPDO mapping starts here; This must be in line with device
  // side TPDOs

  // Master side rpdo1 mapping
  
  device->add_receive_pdo_mapping(0x180 + node_id, "position_actual_value",
                                  0);                                 // 32bit

  device->add_receive_pdo_mapping(0x180 + node_id, "statusword", 4);  // 16bit  

  device->add_receive_pdo_mapping(0x180 + node_id, "current_actual_value",
                                  6);                                 // 16bit

  // Master side RPDO mapping ends here

  /***************** TPDO MAPPING in DEVICE *****************/
  /// Device side TPDO mapping starts here. This must be in line with the
  /// master side RPDOs.

  // Device side tpdo1 mapping entries and mapping
 std::vector<uint32_t> tpdo1_entries_to_be_mapped = {0x60640020, 0x60410010, 0x60780010};

  map_tpdo_in_device(TPDO_1, tpdo1_entries_to_be_mapped, 255, device);


  // Device side tpdo2 mapping entries and mapping
  std::vector<uint32_t> tpdo2_entries_to_be_mapped = {0x60640020, 0x60410010, 0x60780010};
  
  map_tpdo_in_device(TPDO_2, tpdo2_entries_to_be_mapped, 255, device);

  /// Device side TPDO mapping ends here;

  
  /***************** RPDO MAPPING in DEVICE *****************/
  /// Device side RPDO mapping starts here.This must be in line with the
  /// master side TPDOs.
  std::vector<uint32_t> rpdo1_entries_to_be_mapped = {
      0x20300010, 0x60400010,
  };
  map_rpdo_in_device(RPDO_1, rpdo1_entries_to_be_mapped, 255, device);
  /// Device side RPDO mapping ends here

  // Try to clear all possible errors in the CANOpen device
  device->set_entry("controlword", static_cast<uint16_t>(0x0080),
                    kaco::WriteAccessMethod::sdo);
  
}


using namespace std::chrono;

void chatterCallback(const std_msgs::String::ConstPtr& msg){
     std::cout<<"HELLO";
     }

int main(int argc, char* argv[]) {

  // Create bridge
  ros::init(argc, argv, "hexa_node3");
  kaco::Bridge bridge;
  ros::NodeHandle n;

  // Signal handleing
  signal(SIGINT, intHandler);
  
  // Define Publisher
  ros::Publisher chatter_pub0 = n.advertise<std_msgs::UInt16>("actualCurrent0", 1000);
  ros::Publisher chatter_pub1 = n.advertise<std_msgs::UInt16>("actualCurrent1", 1000);
  

  // ----------- //
  // Preferences //
  // ----------- //

  // A Roboteq motor driver with firmware version v2.0beta07032018 was used to
  // test this program.//

  // The node ID of the slave we want to communicate with.
  const uint8_t node_id_0 = 7;
  const uint8_t node_id_1 = 1;

  const std::string busname = "can0";

  const std::string baudrate = "1M";

  const uint16_t heartbeat_interval = 2000;

  // Set the heartbeat time out, after which the system should detect slave
  // disconnection; values can be "250", "500", "1000" and "2000" millisecond.
  // Temporary disabled the timeout parameters; A gloabl 2 second time is now
  // used in device_alive and device_dead callback
  // const uint16_t heartbeat_timeout = heartbeat_interval * 3;

  // -------------- //
  // Initialization //
  // -------------- //

  // Create core.
  kaco::Core core;
  volatile bool found_node = false;
  volatile bool device_connected = false;
  std::mutex device_mutex;
  volatile bool found_node_1 = false;
  volatile bool device_connected_1 = false;
  std::mutex device_mutex_1;

  std::cout << "Starting Core (connect to the driver and start the receiver "
               "thread)...Hi"
            << std::endl;
  if (!core.start(busname, baudrate)) {
    std::cout << "Starting core failed." << std::endl;
    return EXIT_FAILURE;
  }
  // Create device pointer
  std::shared_ptr<kaco::Device> device;
  std::shared_ptr<kaco::Device> device_1;
  // This will be set to true by the callback below.

  std::cout << "Registering a callback which is called when a device is "
               "detected via NMT..."
            << std::endl;
  // make sure that the node is reset and goes back to NMT preoperational
  core.nmt.send_nmt_message(node_id_0, kaco::NMT::Command::reset_node);
  core.nmt.send_nmt_message(node_id_1, kaco::NMT::Command::reset_node);
  core.nmt.register_device_alive_callback([&](const uint8_t new_node_id) {
    // Check if this is the node we are looking for.
    if (new_node_id == node_id_0) {
      // lock
      if (!found_node) {
        found_node = true;
        // Lock device mutex
        std::lock_guard<std::mutex> lock(device_mutex);
        try {
          // Initialize the device
          device.reset(new kaco::Device(core, node_id_0));
          std::string path = ros::package::getPath("kacanopen");

          boost::filesystem::path full_path = "/home/pi/hexa_workspace/src/hexa_package/edsFiles/epos2_347717.eds";    
              
          device->load_dictionary_from_eds(full_path.string());
          std::cout << "Printing Device Object Dictionary" << std::endl;
          device->print_dictionary();
          core.nmt.send_nmt_message(node_id_0,
                                    kaco::NMT::Command::enter_preoperational);
          
          initializeDevice(device, heartbeat_interval, node_id_0);
          int8_t set_mode_of_operation = -3;
                
          std::this_thread::sleep_for(std::chrono::microseconds(5));

          
          device->set_entry(0x6060, 0x00, set_mode_of_operation,
                           kaco::WriteAccessMethod::sdo);

          device->start();
          printDeviceInfo(device);
          device_connected = true;

        } catch (const std::exception &exc) {
          std::cout << "Exception in device alive!" << std::endl;
          std::cerr <<exc.what();
          found_node = false;
          device_connected = false;
        }
      }
    }
    if (new_node_id == node_id_1) {
      // lock
      if (!found_node_1) {
        found_node_1 = true;
        // Lock device mutex
        std::lock_guard<std::mutex> lock(device_mutex);
        try {
          // Initialize the device
          device_1.reset(new kaco::Device(core, node_id_1));
          std::string path = ros::package::getPath("kacanopen");
        
          boost::filesystem::path full_path = "/home/pi/hexa_workspace/src/hexa_package/edsFiles/epos2_347717.eds";    
              
          device_1->load_dictionary_from_eds(full_path.string());
          std::cout << "Printing Device Object Dictionary" << std::endl;
          core.nmt.send_nmt_message(node_id_1,
                                    kaco::NMT::Command::enter_preoperational);
          
          initializeDevice(device_1, heartbeat_interval, node_id_1);
          
          int8_t set_mode_of_operation = -3;  
          
          std::this_thread::sleep_for(std::chrono::microseconds(100));

          device_1->set_entry(0x6060, 0x00, set_mode_of_operation,
                           kaco::WriteAccessMethod::sdo);
          
          device_1->start();
          printDeviceInfo(device_1);
          device_connected_1 = true;

        } catch (const std::exception &exc) {
          std::cout <<  "Exception in device alive!" << std::endl;
          std::cerr <<  exc.what();
          found_node_1 = false;
          device_connected_1 = false;
        }
      }
    }
  });
  
  int loopCounter = 0;
  high_resolution_clock::time_point lastTime = high_resolution_clock::now();

  high_resolution_clock::time_point startprogramm = high_resolution_clock::now();
  while (keepRunning) {
    high_resolution_clock::time_point startloop = high_resolution_clock::now();
    if (device_connected && device_connected_1) {
     
      // Lock device mutex
      std::lock_guard<std::mutex> lock(device_mutex);

      try {
        
        duration<double, std::milli> time_take_to_this = high_resolution_clock::now() - startprogramm;
        double currentTime = time_take_to_this.count() ;
        int16_t actual_curr_0 =
            device->get_entry("current_actual_value",
                              kaco::ReadAccessMethod::pdo_request_and_wait);

        int16_t actual_curr_1 =
            device_1->get_entry("current_actual_value",
                              kaco::ReadAccessMethod::pdo_request_and_wait);
              
        duration<double, std::milli> time_span = high_resolution_clock::now() - lastTime;

        lastTime = high_resolution_clock::now();
        
        device_1->set_entry("current_mode_setting_value", static_cast<int16_t>(1000),
                          kaco::WriteAccessMethod::pdo);

        device->set_entry("controlword", static_cast<uint16_t>(0x000F),
                          kaco::WriteAccessMethod::pdo);

        device_1->set_entry("controlword", static_cast<uint16_t>(0x000F),
                          kaco::WriteAccessMethod::pdo);

        loopCounter = loopCounter + 1;
        
        
        // Publish This Message
        std_msgs::UInt16 msg0;
        msg0.data = actual_curr_0;
        chatter_pub0.publish(msg0);
        
        std_msgs::UInt16 msg1;
        msg1.data = actual_curr_1;
        chatter_pub1.publish(msg1);
        

      } catch (const std::exception &exc) {        
        std::cout << "Exception in main!" << std::endl;
        std::cerr <<exc.what();
      }
    }
    duration<double, std::milli> loop_calculation_time_span = high_resolution_clock::now() - startloop;
    // Sleep
    if((unsigned int)loop_calculation_time_span.count() < 5 ){
      std::this_thread::sleep_for(std::chrono::milliseconds(5 - (unsigned int)loop_calculation_time_span.count() ));
    }else{
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
};
