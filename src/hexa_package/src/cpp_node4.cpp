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

#include "hexa_package/drivesFeedBack.h"
#include "hexa_package/drivesAction.h"
#include "hexa_package/sensor.h"
#include "hexa_package/setting.h"

#include <chrono>
#include <memory>
#include <thread>




/* For setting the process's priority (setpriority) */
#include <sys/resource.h>
/* For pid_t and getpid() */
#include <unistd.h>
#include <sys/types.h>
/* For locking the program in RAM (mlockall) to prevent swapping */
#include <sys/mman.h>
/* clock_gettime, struct timespec, etc. */
#include <time.h>



using namespace std;
void control();
static volatile int keepRunning = 1;

void intHandler(int dummy) {
  (void)dummy;
  keepRunning = 0;

  printf("trying to stop process %d\n",getpid());
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  /*do something*/
  printf("killing process %d\n",getpid());
  ros::shutdown();

  //TODO check device available and then set current    
  // Currentleft = 0;
  // Currentright = 0;
  // device_1->set_entry("current_mode_setting_value", Currentleft, kaco::WriteAccessMethod::pdo);
  // device->set_entry("current_mode_setting_value", Currentright, kaco::WriteAccessMethod::pdo);
  
  exit(0);
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
       kaco::TransmissionType::ON_CHANGE, std::chrono::milliseconds(5)); //kaco::TransmissionType::PERIODIC - ON_CHANGE
  
  // Master side RPDO mapping starts here; This must be in line with device
  // side TPDOs

  // Master side rpdo1 mapping
  
  device->add_receive_pdo_mapping(0x180 + node_id, "position_actual_value", 0);                                 // 32bit

  device->add_receive_pdo_mapping(0x180 + node_id, "statusword", 4);  // 16bit  

  device->add_receive_pdo_mapping(0x180 + node_id, "current_actual_value", 6);                                 // 16bit

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

// Create device pointer
std::shared_ptr<kaco::Device> device;
std::shared_ptr<kaco::Device> device_1;


volatile bool found_node = false;
volatile bool device_connected = false;
std::mutex device_mutex;
volatile bool found_node_1 = false;
volatile bool device_connected_1 = false;
std::mutex device_mutex_1;



bool messageRecievedFromController = false;

high_resolution_clock::time_point lastTime = high_resolution_clock::now();



// void chatterCallback0(const std_msgs::Int16::ConstPtr& msg){
//   duration<double, std::milli> time_take_to_this = high_resolution_clock::now() - lastTime;
//   double currentTime = time_take_to_this.count() ;
//   lastTime = high_resolution_clock::now();
//   messageRecievedFromController = true;
//   if(device_connected){    
//     cout << msg->data << "\t" << currentTime << "\n" ;
//     device->set_entry("current_mode_setting_value", msg->data,
//                             kaco::WriteAccessMethod::pdo);
//     // device->set_entry("controlword", static_cast<uint16_t>(0x000F),
//     //                         kaco::WriteAccessMethod::pdo);
//   }
// }

// void chatterCallback1(const std_msgs::Int16::ConstPtr& msg){
//   messageRecievedFromController = true;
//   if(device_connected_1){
//     device_1->set_entry("current_mode_setting_value", msg->data,
//                             kaco::WriteAccessMethod::pdo);
//   }
// }




// void actionSubscriberCallback(const hexa_package::drivesAction::ConstPtr& msg){
//   duration<double, std::milli> time_take_to_this = high_resolution_clock::now() - lastTime;
//   double currentTime = time_take_to_this.count() ;
//   lastTime = high_resolution_clock::now();
//   messageRecievedFromController = true;
//   if(currentTime > 6 || currentTime < 1)
//   cout << currentTime << "\n" ;
//   if(device_connected && device_connected_1) {
//     // cout << msg->current0;
//     device->set_entry("current_mode_setting_value", msg->current0, kaco::WriteAccessMethod::pdo);
//     device_1->set_entry("current_mode_setting_value", msg->current1, kaco::WriteAccessMethod::pdo);
//   }
  
// }


#define MAX_SAFE_STACK (8 * 1024)

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

int16_t Currentleft = 0;
int16_t Currentright = 0;
int32_t LoadcellRightHip = 0;
int32_t LoadcellLeftHip = 0;

int16_t actual_curr_0 = 0;
int16_t actual_curr_1 = 0;
int32_t actual_position_0 = 0;
int32_t actual_position_1 = 0;
int32_t P_RH = 50;//30;
int32_t P_LH = 50;//30;

void getSensorData(const hexa_package::sensor::ConstPtr& msg){
  //cout  << endl << msg->loadcell1 << endl;
 
  duration<double, std::milli> time_take_to_this = high_resolution_clock::now() - lastTime;
  double currentTime = time_take_to_this.count() ;
  lastTime = high_resolution_clock::now();
  //Todo print time for debug
  // cout << currentTime << "\n" ;
  LoadcellRightHip = msg->loadcell1;
  LoadcellLeftHip = msg->loadcell0;
}


void getSettingData(const hexa_package::setting::ConstPtr& msg){
  std::cout<<"GUI Settings"<<endl << msg->right_assistive_force << endl<<msg->left_assistive_force<<endl;
  P_RH = msg->right_assistive_force;
  P_LH = msg->left_assistive_force;
}



int main(int argc, char* argv[]) {





        cpu_set_t set;
        //CPU_ZERO(&set);
        //CPU_SET(3, &set);

        //if (sched_setaffinity(0, sizeof(set), &set))
        //{
         //       printf("Setting CPU affinity failed!\n");
          //      return -1;
        //}
        struct sched_param param = {};
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        printf("Using priority %i.\n", param.sched_priority);
        if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
        {
                perror("sched_setscheduler failed\n");
        }

        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
        {
                printf("mlockall failed\n");
                return -1;
        }

        stack_prefault();





  // Create bridge
  ros::init(argc, argv, "hexa_node4");
  kaco::Bridge bridge;
  ros::NodeHandle n;

  //ros::AsyncSpinner spinner(4); // Use 2 threads
  //spinner.start();
  //ros::waitForShutdown();

  ros::Rate loop_rate(200);




  // Signal handleing
  signal(SIGINT, intHandler);
  
  // Define Publisher
  // ros::Publisher chatter_pub0 = n.advertise<std_msgs::Int16>("actualCurrent0", 1000);
  // ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int16>("actualCurrent1", 1000);
  // ros::Publisher postion_message_publisher_0 = n.advertise<std_msgs::Int32>("actualPosition0", 1000);
  // ros::Publisher postion_message_publisher_1 = n.advertise<std_msgs::Int32>("actualPosition1", 1000);
  ros::Publisher drivesFeedBack_publisher = n.advertise<hexa_package::drivesFeedBack>("drivesFeedBack" , 10000);

  // ----------- //
  // Preferences //
  // ----------- //

  // The node ID of the slave we want to communicate with.
  const uint8_t node_id_0 = 7;
  const uint8_t node_id_1 = 1;

  const std::string busname = "can0";

  const std::string baudrate = "1M";

  const uint16_t heartbeat_interval = 1000;

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


  std::cout << "Starting Core (connect to the driver and start the receiver "
               "thread)...Hi"
            << std::endl;
  if (!core.start(busname, baudrate)) {
    std::cout << "Starting core failed." << std::endl;
    return EXIT_FAILURE;
  }
  
  // This will be set to true by the callback below.

  std::cout << "Registering a callback which is called when a device is "
               "detected via NMT..."
            << std::endl;
  // make sure that the node is reset and goes back to NMT preoperational
  core.nmt.send_nmt_message(node_id_0, kaco::NMT::Command::reset_node);
  core.nmt.send_nmt_message(node_id_1, kaco::NMT::Command::reset_node);
  core.nmt.register_device_alive_callback([&](const uint8_t new_node_id) {
    cout << "node found \n";
    // Check if this is the node we are looking for.
    if (new_node_id == node_id_0) {
      // lock
      if (!found_node) {
        found_node = true;
        // Lock device mutex 
        //TODO check why?
        std::lock_guard<std::mutex> lock(device_mutex);
        try {
          // Initialize the device
          device.reset(new kaco::Device(core, node_id_0));
          std::string path = ros::package::getPath("kacanopen");

          boost::filesystem::path full_path = "/home/pi/hexa_workspace/src/hexa_package/edsFiles/epos2_347717.eds";    
              
          device->load_dictionary_from_eds(full_path.string());
          //std::cout << "Printing Device Object Dictionary" << std::endl;
          //device->print_dictionary(); // this print all eds dictionary
          core.nmt.send_nmt_message(node_id_0,
                                    kaco::NMT::Command::enter_preoperational);
          
          initializeDevice(device, heartbeat_interval, node_id_0);
          int8_t set_mode_of_operation = -3;
                
          std::this_thread::sleep_for(std::chrono::microseconds(5000));

          
          device->set_entry(0x6060, 0x00, set_mode_of_operation,
                           kaco::WriteAccessMethod::sdo);

          device->start();
          //printDeviceInfo(device);
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
        std::lock_guard<std::mutex> lock(device_mutex_1);
        try {
          // Initialize the device
          device_1.reset(new kaco::Device(core, node_id_1));
          std::string path = ros::package::getPath("kacanopen");
        
          boost::filesystem::path full_path = "/home/pi/hexa_workspace/src/hexa_package/edsFiles/epos2_347717.eds";    
              
          device_1->load_dictionary_from_eds(full_path.string());
          core.nmt.send_nmt_message(node_id_1,
                                    kaco::NMT::Command::enter_preoperational);
          
          initializeDevice(device_1, heartbeat_interval, node_id_1);
          
          int8_t set_mode_of_operation = -3;  
          
          std::this_thread::sleep_for(std::chrono::microseconds(5000));

          device_1->set_entry(0x6060, 0x00, set_mode_of_operation,
                           kaco::WriteAccessMethod::sdo);
          
          device_1->start();
          //printDeviceInfo(device_1);
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


  std::cout<<"start subscring .. .. . ."<<endl;
  ros::Subscriber loadCell1Sub = n.subscribe("sensor", 1000, getSensorData);
  ros::Subscriber settingsSub = n.subscribe("setting", 1000, getSettingData);

  // Define a lamda expression
auto f = []() {
    ros::spin();
    //or
    //ros::spinOnce();
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));

};
  
// Pass f and its parameters to thread 
// object constructor as
std::thread thread_object(f);


  while (!(device_connected && device_connected_1)) {
    cout << "waiting for drives\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }


  //waite for sensor data 

  while (!(LoadcellRightHip != 0 && LoadcellLeftHip != 0)) {
    cout << "waiting for sensor data\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }



  //ros::Subscriber actionSubscriber = n.subscribe("drivesAction", 1000, actionSubscriberCallback);


        device->set_entry("controlword", static_cast<uint16_t>(0x000F),
                           kaco::WriteAccessMethod::pdo);

        device_1->set_entry("controlword", static_cast<uint16_t>(0x000F),
                           kaco::WriteAccessMethod::pdo);

  cout << "\noperational\n";
  while (keepRunning) {
    high_resolution_clock::time_point startloop = high_resolution_clock::now();
    if (device_connected && device_connected_1) {
      
      // Lock device mutex
      //todo check why?
      std::lock_guard<std::mutex> lock(device_mutex);

      try {
        
        // duration<double, std::milli> time_take_to_this = high_resolution_clock::now() - startprogramm;
        // double currentTime = time_take_to_this.count() ;
        actual_curr_0 =
            device_1->get_entry("current_actual_value",
                              kaco::ReadAccessMethod::cache);
        //pdo_request_and_wait
        actual_curr_1 =
            device->get_entry("current_actual_value",
                              kaco::ReadAccessMethod::cache);

        actual_position_0 =
            device_1->get_entry("position_actual_value",
                              kaco::ReadAccessMethod::cache);

        actual_position_1 =
            device->get_entry("position_actual_value",
                              kaco::ReadAccessMethod::cache);

        //cout << actual_curr_0 << " " << actual_curr_1 << "\n";
        //cout << actual_position_0 << " " << actual_position_1 << "\n";
        duration<double, std::milli> time_span = high_resolution_clock::now() - lastTime;

        lastTime = high_resolution_clock::now();
        
        // device_1->set_entry("current_mode_setting_value", static_cast<int16_t>(1000),
        //                   kaco::WriteAccessMethod::pdo);

        // device_1->set_entry("current_mode_setting_value", desiredCurrent,
        //                   kaco::WriteAccessMethod::pdo);

        // device_1->set_entry("current_mode_setting_value", static_cast<int16_t>(1000),
        //                   kaco::WriteAccessMethod::pdo);

        // device->set_entry("controlword", static_cast<uint16_t>(0x000F),
        //                   kaco::WriteAccessMethod::pdo);

        // device_1->set_entry("controlword", static_cast<uint16_t>(0x000F),
        //                   kaco::WriteAccessMethod::pdo);

        loopCounter = loopCounter + 1;
        
        
        // Publish This Message
        // std_msgs::Int16 msg0;
        // msg0.data = actual_curr_0;
        // chatter_pub0.publish(msg0);
        
        // std_msgs::Int16 msg1;
        // msg1.data = actual_curr_1;
        // chatter_pub1.publish(msg1);

        // static int32_t actual_position_0_old = -970;   
        // static int32_t actual_position_1_old = -970;
        // // Publish postion message
        // if(messageRecievedFromController && actual_position_0_old != actual_position_0 ){
        //   actual_position_0_old = actual_position_0;
        //   std_msgs::Int32 postionMessage0;
        //   postionMessage0.data = actual_position_0;
        //   postion_message_publisher_0.publish(postionMessage0);
        // }


        // if(messageRecievedFromController && actual_position_1_old != actual_position_1){
        //   actual_position_1_old = actual_position_1;
        //   std_msgs::Int32 postionMessage1;
        //   postionMessage1.data = actual_position_1;
        //   postion_message_publisher_1.publish(postionMessage1);
        // }



        // hexa_package::drivesFeedBack  drivesFeedBack;
        // drivesFeedBack.actualPosition0 = actual_position_0;
        // drivesFeedBack.actualPosition1 = actual_position_1;
        // drivesFeedBack.actualCurrent0 = actual_curr_0;
        // drivesFeedBack.actualCurrent1 = actual_curr_1;

        // drivesFeedBack_publisher.publish(drivesFeedBack);


        control();



        device_1->set_entry("current_mode_setting_value", Currentleft, kaco::WriteAccessMethod::pdo);
        device->set_entry("current_mode_setting_value", Currentright, kaco::WriteAccessMethod::pdo);

      } catch (const std::exception &exc) {        

        std::cout << "Exception in main!" << std::endl;
        std::cerr <<exc.what();
      }
    }

    //turn off device
    Currentleft = 0;
    Currentright = 0;
    device_1->set_entry("current_mode_setting_value", Currentleft, kaco::WriteAccessMethod::pdo);
    device->set_entry("current_mode_setting_value", Currentright, kaco::WriteAccessMethod::pdo);

    // duration<double, std::milli> loop_calculation_time_span = high_resolution_clock::now() - startloop;
    // // Sleep
    // if((unsigned int)loop_calculation_time_span.count() < 5 ){
    //   std::this_thread::sleep_for(std::chrono::milliseconds(5 - (unsigned int)loop_calculation_time_span.count() ));
    // }else{
    //   std::this_thread::sleep_for(std::chrono::microseconds(100));
    // }
    // ros::spinOnce();
    loop_rate.sleep();
    //ros::Rate.sleep();
  }

  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
};



void control(){



static long double a_filter = 0.1;
static long double pi = 3.14;
static long double D_Time;
static long double D_time = D_Time = 0.005;
static long double Current_Limit = 3000;

static long double timer = 0;
static long double Assist_R = 0;
static long double Assist_L = 0;

//# Feedback
// long double LoadcellRightHip = 0;
// long double LoadcellLeftHip = 0;
static long double PositionActualValueright = actual_position_1;
static long double PositionActualValueleft = actual_position_0;
static long double VelocityActualValueright = 0;
static long double VelocityActualValueleft = 0;

static long double CurrentActualValueright = actual_curr_1;
static long double CurrentActualValueleft = actual_curr_0;


static long double Zero_Impedance_Gain = 1;

static long double I_R = 0.1;
static long double C_R = 1.5;
static long double FK_R = 1.5;
static long double MgL_R =0.5;
static long double PID_Gain_R = 0.3;
static long double N_RH = 100;

static long double I_L = 0.1;
static long double C_L = 1.5;
static long double FK_L = 1.5;
static long double MgL_L = 0.5;
static long double PID_Gain_L = 0.15;
static long double N_LH = 100;

//# initial value


long double Flt_Vel_RH_old = 0;
long double Flt_Vel_LH_old = 0;
long double L_Assist = 755;
long double R_Assist = 750;
long double R_o = 0;
long double L_o = 0;
long double Flt_Acc_RH_old = 0;
long double Flt_Acc_LH_old = 0;
long double ef_RH_old = 0;
long double ef_LH_old = 0;
long double PD_RH_old = 0;
long double PD_LH_old = 0;

long double D_RH = 0.1;
long double D_LH = 0.1;
long double timer_r = 0;
long double timer_l = 0;




//rospy.init_node('control', anonymous=True)
//rate = rospy.Rate(200)  # 200hz

//# Publish
//# desiredCurrentRight = rospy.Publisher('desiredCurrent0', Int16, queue_size=1000)
//# desiredCurrentLeft = rospy.Publisher('desiredCurrent1', Int16, queue_size=1000)

//drives_action = rospy.Publisher('drivesAction', drivesAction, queue_size=1)


//# Subscribe


// def get_load_cell1(message):
//     global LoadcellRightHip
//     LoadcellRightHip = message.data
// #    print("load cell right: " + str(LoadcellRightHip))

// def get_load_cell2(message):
//     global LoadcellLeftHip
//     LoadcellLeftHip = message.data
// #    print("load cell left: " + str(LoadcellLeftHip))



// def get_setting(message):
//     data = message_converter.convert_ros_message_to_dictionary(message)
//     print(data)
//     global P_RH
//     P_RH = data['right_assistive_force']
//     global P_LH
//     P_LH = data['left_assistive_force']


// def get_drive_feedback(message):
//     global PositionActualValueleft
//     global PositionActualValueright
//     global CurrentActualValueleft
//     global CurrentActualValueright

//     PositionActualValueleft = message.actualPosition0
//     PositionActualValueright = message.actualPosition1

//     CurrentActualValueleft = message.actualCurrent0
//     CurrentActualValueright = message.actualCurrent1
//     print("time:" + str(time.time()) )


// rospy.Subscriber('loadcell1', UInt16, get_load_cell1)
// rospy.Subscriber('loadcell2', UInt16, get_load_cell2)
// # rospy.Subscriber('actualPosition0', Int32, get_position_actual_right)
// # rospy.Subscriber('actualPosition1', Int32, get_position_actual_left)

// # rospy.Subscriber('actualCurrent0', Int16, get_current_actual_right)
// # rospy.Subscriber('actualCurrent1', Int16, get_current_actual_left)

// rospy.Subscriber('drivesFeedBack', drivesFeedBack, get_drive_feedback)
// #rospy.Subscriber('setting', setting, send_setting)
// loopCounter = 0
// while not rospy.is_shutdown():
    long double Force_RH = 0.07*(- 0.023306 * (LoadcellRightHip) + 458.15 - 6.0) - 6.75;
    long double Force_LH = 0.07*(- 0.023359 * (LoadcellLeftHip) + 756.79 - 6.0);

    //# Theta
    long double Theta_RH = -(2 * pi / (4800)) * PositionActualValueright;
    long double Theta_LH = +(2 * pi / (4800)) * PositionActualValueleft;

    //# Position Filter
    long double Flt_Vel_RH = (1 - D_Time / a_filter) * Flt_Vel_RH_old + Theta_RH * D_Time / a_filter;
    long double Flt_Vel_LH = (1 - D_Time / a_filter) * Flt_Vel_LH_old + Theta_LH * D_Time / a_filter;

    //# Velocity
    long double Theta_dt_RH = -2 * pi * VelocityActualValueright / (60 * 100);
    long double Theta_dt_LH = +2 * pi * VelocityActualValueleft / (60 * 100);

    long double Velocity_RH = (Flt_Vel_RH - Flt_Vel_RH_old) / D_Time;
    long double Velocity_LH = (Flt_Vel_LH - Flt_Vel_LH_old) / D_Time;

    Flt_Vel_RH_old = Flt_Vel_RH;
    Flt_Vel_LH_old = Flt_Vel_LH;

    //# M*g*sin(Theta)
    long double mgsin_RH = 0.5 * 5.8 * 9.81 * 0.341 * sin(Theta_RH);
    long double mgsin_LH = 0.5 * 5.8 * 9.81 * 0.341 * sin(Theta_LH);

    //# Velocity Filter
    long double Flt_Acc_RH = (1 - D_Time / a_filter) * Flt_Acc_RH_old + Theta_dt_RH * D_Time / a_filter;
    long double Flt_Acc_LH = (1 - D_Time / a_filter) * Flt_Acc_LH_old + Theta_dt_LH * D_Time / a_filter;

    //# Acceleration
    long double ACC_RH = (Flt_Acc_RH - Flt_Acc_RH_old) / D_Time;
    long double ACC_LH = (Flt_Acc_LH - Flt_Acc_LH_old) / D_Time;

    Flt_Acc_RH_old = Flt_Acc_RH;
    Flt_Acc_LH_old = Flt_Acc_LH;

    //# I*Acceleration
    long double I_Theta2dot_RH = 1.61 * ACC_RH;
    long double I_Theta2dot_LH = 1.61 * ACC_LH;

    long double SIGN_Theta_dt_RH = 0;
    //# Identification
    if (Force_RH > 0){
        SIGN_Theta_dt_RH = -1;
    } else if (Force_RH < 0){
        SIGN_Theta_dt_RH = +1;
    }

    long double SIGN_Theta_dt_LH = 0;
    if (Force_LH > 0){
        SIGN_Theta_dt_LH = -1;
    } else if ( Force_LH < 0){
        SIGN_Theta_dt_LH = +1;
    }

    //# PD Code
    long double ef_RH = 0 - Force_RH;
    long double ef_LH = 0 - Force_LH;

    long double ef_RH_dt = (ef_RH - ef_RH_old) / D_time;
    long double ef_LH_dt = (ef_LH - ef_LH_old) / D_time;

    long double PD_RH = (1 - N_RH * D_time) * PD_RH_old + N_RH * P_RH * D_time * (ef_RH) + (P_RH + N_RH * D_RH) * D_time * ef_RH_dt;
    long double PD_LH = (1 - N_LH * D_time) * PD_LH_old + N_LH * P_LH * D_time * (ef_LH) + (P_LH + N_LH * D_LH) * D_time * ef_LH_dt;

    ef_RH_old = ef_RH;
    ef_LH_old = ef_LH;

    PD_RH_old = PD_RH;
    PD_LH_old = PD_LH;

    //# Zero Impedance
    long double Trq_RH = Zero_Impedance_Gain * (I_R * ACC_RH + C_R * Theta_dt_RH + FK_R * SIGN_Theta_dt_RH + MgL_R * sin(Theta_RH) + PID_Gain_R * PD_RH);
    long double Trq_LH = Zero_Impedance_Gain * (I_L * ACC_LH + C_L * Theta_dt_LH + FK_L * SIGN_Theta_dt_LH + MgL_L * sin(Theta_LH) + PID_Gain_L * PD_LH);

    //# ASSIST
    if (timer_r > timer){
        R_o = 0;
    }else if (Velocity_RH > 0 and Theta_RH <= 1.2 and Velocity_LH <= 0){
        R_o = 1;
    }

    if (timer_l > timer){
        L_o = 0;
    }
    else if ( Velocity_LH > 0 and Theta_LH <= 1.2 and Velocity_RH <= 0){
        L_o = 1;
    }

    if (R_o == 1){
        timer_r = timer_r + 1;
        R_Assist = Assist_R;
    }
    else if (R_o == 0){
        timer_r = 0;
        R_Assist = 0;
    }

    if (L_o == 1){
        timer_l = timer_l + 1;
        L_Assist = Assist_L;
    }
    else if (L_o == 0){
        timer_l = 0;
        L_Assist = 0;
    }

    Trq_RH = -(Trq_RH/0.0369 + 6 * R_Assist);
    Trq_LH = +(Trq_LH/0.0369 + 6 * L_Assist);

    //# Torque Limit
    if (Trq_RH > Current_Limit){
        Trq_RH = Current_Limit;
    }
    else if (Trq_RH < -Current_Limit){
        Trq_RH = -Current_Limit;
    }

    if (Trq_LH > Current_Limit){
        Trq_LH = Current_Limit;
    }
    else if (Trq_LH < -Current_Limit){
        Trq_LH = -Current_Limit;
    }

    // # Send TO Derive
    Currentright = (1 * Trq_RH);
    Currentleft = (1 * Trq_LH);
    // #print("Currentright " + str(Currentright));
    // #print("Currentleft " + str(Currentleft));
    static int loopCounter;
    loopCounter ++;
    //Currentright = 200 * sin(loopCounter / 200.00);
    //Currentleft = 200 * sin(loopCounter / 200.00);

    // # desiredCurrentRight.publish(int(Currentright))
    // # desiredCurrentLeft.publish(int(Currentleft))

    // # drivesFeedBack df
    // # df.actualCurrent0 = Currentleft
    // # df.actualCurrent1 = Currentright
    // drives_action.publish(drivesAction(int(Currentleft), int(Currentright)))

    // #desiredCurrentRight.publish(int(500))
    // #desiredCurrentLeft.publish(int(500))
    // # if(loopCounter % 1 == 0):
    // #     print("time:" + str(time.time()) + "\tForce_LH:" + str(Force_LH) + "\tForce_RH:" + str(Force_RH) +"\tCurrentleft:" + str(Currentleft) + "\tCurrentright:" + str(Currentright) + "\tCurrentActualValueright:" + str(CurrentActualValueright) + "\tCurrentActualValueleft" + str(CurrentActualValueleft) + "\tLoadcellLeftHip:" + str(LoadcellLeftHip) + "\tLoadcellRightHip:" + str(LoadcellRightHip) + "\tPositionActualValueleft:" + str(PositionActualValueleft)  + "\tPositionActualValueright:" + str(PositionActualValueright)) 
    // loopCounter = loopCounter + 1
    // #print("time:" + str(time.time()) )
    //rate.sleep()
    //  if(loopCounter % 40)
     //  cout << "time:" << "str(time.time())" << "\tForce_LH:" << (Force_LH) << "\tForce_RH:" << (Force_RH) << "\tCurrentleft:" <<(Currentleft) << "\tCurrentright:" << (Currentright) << "\tCurrentActualValueright:" << (CurrentActualValueright) << "\tCurrentActualValueleft" << (CurrentActualValueleft) << "\tLoadcellLeftHip:" << (LoadcellLeftHip) << "\tLoadcellRightHip:" << (LoadcellRightHip) << "\tPositionActualValueleft:" << (PositionActualValueleft)  << "\tPositionActualValueright:" << (PositionActualValueright) << endl ;
      //  cout << Force_LH << "\tForce_RH:" << (Force_RH) << "\n";
// cout << LoadcellRightHip;

  // duration<double, std::milli> time_take_to_this = high_resolution_clock::now() - lastTime;
  // double currentTime = time_take_to_this.count() ;
  // lastTime = high_resolution_clock::now();
  // cout << currentTime << "\n" ;


}