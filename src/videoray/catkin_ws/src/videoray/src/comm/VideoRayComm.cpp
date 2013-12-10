#include <iostream>
#include <stdio.h>

#include "VideoRayComm.h"

//////////////////////////////
// TX Control Packet Defines
//////////////////////////////
#define PORT_THRUST_LSB  0
#define PORT_THRUST_MSB  1
#define STAR_THRUST_LSB  2
#define STAR_THRUST_MSB  3
#define VERT_THRUST_LSB  4
#define VERT_THRUST_MSB  5
#define LIGHTS_LSB       6
#define CAM_TILT         7
#define CAM_FOCUS        8
#define UNKNOWN_0        9
#define UNKNOWN_1        10
#define AUTO_DEPTH_LSB   11
#define AUTO_DEPTH_MSB   12
#define AUTO_HEADING_LSB 13
#define AUTO_HEADING_MSB 14

#define TX_CTRL_SIZE     15

////////////////////////////////
//// RX Control Packet Defines
////////////////////////////////
//#define UNKNOWN_2   0
//#define HEADING_LSB 1
//#define HEADING_MSB 2
//#define PITCH_LSB   3
//#define PITCH_MSB   4
//#define ROLL_LSB    5
//#define ROLL_MSB    6

//////////////////////////////
// RX Control Packet Defines
// CSR Map Defines
////////////////////////////
//#define DEVICE_ID      0
//#define DEPTH_LSB      1
//#define DEPTH_MSB      2
//#define HEADING_LSB    3
//#define HEADING_MSB    4
//#define PITCH_LSB      5
//#define PITCH_MSB      6
//#define ROLL_LSB       7
//#define ROLL_MSB       8
//#define YAW_ACC_LSB    9
//#define YAW_ACC_MSB    10
//#define PITCH_ACC_LSB  11
//#define PITCH_ACC_MSB  12
//#define ROLL_ACC_LSB   13
//#define ROLL_ACC_MSB   14
//#define SURGE_ACC_LSB  15
//#define SURGE_ACC_MSB  16
//#define SWAY_ACC_LSB   17
//#define SWAY_ACC_MSB   18
//#define HEAVE_ACC_LSB  19
//#define HEAVE_ACC_MSB  20

#define DEVICE_ID      0
#define HEADING_LSB    1
#define HEADING_MSB    2
#define PITCH_LSB      3
#define PITCH_MSB      4
#define ROLL_LSB       5
#define ROLL_MSB       6
#define DEPTH_LSB      7
#define DEPTH_MSB      8
#define YAW_ACC_LSB    9
#define YAW_ACC_MSB    10
#define PITCH_ACC_LSB  11
#define PITCH_ACC_MSB  12
#define ROLL_ACC_LSB   13
#define ROLL_ACC_MSB   14
#define SURGE_ACC_LSB  15
#define SURGE_ACC_MSB  16
#define SWAY_ACC_LSB   17
#define SWAY_ACC_MSB   18
#define HEAVE_ACC_LSB  19
#define HEAVE_ACC_MSB  20
#define RAW_MAG_X_LSB  21
#define RAW_MAG_X_MSB  22
#define RAW_MAG_Y_LSB  23
#define RAW_MAG_Y_MSB  24
#define RAW_MAG_Z_LSB  25
#define RAW_MAG_Z_MSB  26
#define ATTITUDE_LSB   27
#define ATTITUDE_MSB   28
#define HUMIDITY_LSB   29
#define HUMIDITY_MSB   30
#define WATER_TEMP_LSB 31
#define WATER_TEMP_MSB 32
#define ROV_PWR_LSB    33
#define ROV_PWR_MSB    34

using std::cout;
using std::endl;

VideoRayComm::VideoRayComm()
{
     packetizer_.set_network_id(0x01);
     
     int status;
     status = serial_.Open("/dev/ttyUSB0", 115200);
     if (status != 1) {
     	  cout << "Error while opening port. Permission problem ?" << endl;
     	  exit(-1);
     }
     serial_.FlushReceiver();

     tx_ctrl_data[PORT_THRUST_LSB] = 0;
     tx_ctrl_data[PORT_THRUST_MSB] = 0;
     tx_ctrl_data[STAR_THRUST_LSB] = 0;
     tx_ctrl_data[STAR_THRUST_MSB] = 0;
     tx_ctrl_data[VERT_THRUST_LSB] = 0;
     tx_ctrl_data[VERT_THRUST_MSB] = 0;
     tx_ctrl_data[LIGHTS_LSB] = 0;
     tx_ctrl_data[CAM_TILT] = 0;
     tx_ctrl_data[CAM_FOCUS] = 0;
     tx_ctrl_data[UNKNOWN_0] = 0;
     tx_ctrl_data[UNKNOWN_1] = 0;
     tx_ctrl_data[AUTO_DEPTH_LSB] = 0xFF;
     tx_ctrl_data[AUTO_DEPTH_MSB] = 0xFF;
     tx_ctrl_data[AUTO_HEADING_LSB] = 0xFF;
     tx_ctrl_data[AUTO_HEADING_MSB] = 0xFF;
     
     heading_ = 0;
     pitch_ = 0;
     roll_ = 0;
}

VideoRayComm::~VideoRayComm()
{
     serial_.Close();
}

VideoRayComm::Status_t VideoRayComm::set_desired_heading(int heading)
{
     if (heading > 360 || heading <= 0) {
          tx_ctrl_data[AUTO_HEADING_LSB] = 0xFF;
          tx_ctrl_data[AUTO_HEADING_MSB] = 0xFF;
     } else {
          tx_ctrl_data[AUTO_HEADING_LSB] = heading & 0x00FF;
          tx_ctrl_data[AUTO_HEADING_MSB] = (heading & 0xFF00) >> 8;
     }

     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_desired_depth(int depth)
{
     if (depth < 0) {
          tx_ctrl_data[AUTO_DEPTH_LSB] = 0xFF;
          tx_ctrl_data[AUTO_DEPTH_MSB] = 0xFF;
     } else {
          tx_ctrl_data[AUTO_DEPTH_LSB] = depth & 0x00FF;
          tx_ctrl_data[AUTO_DEPTH_MSB] = (depth & 0xFF00) >> 8;
     }
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_focus(int focus)
{
     tx_ctrl_data[CAM_FOCUS] = focus;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_camera_tilt(int tilt)
{
     tx_ctrl_data[CAM_TILT] = tilt;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_lights(int lights)
{
     tx_ctrl_data[LIGHTS_LSB] = lights;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_vertical_thruster(int thrust)
{
     tx_ctrl_data[VERT_THRUST_LSB] = thrust & 0x00FF;
     tx_ctrl_data[VERT_THRUST_MSB] = (thrust & 0xFF00) >> 8;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_port_thruster(int thrust)
{
     tx_ctrl_data[PORT_THRUST_LSB] = thrust & 0x00FF;
     tx_ctrl_data[PORT_THRUST_MSB] = (thrust & 0xFF00) >> 8;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_starboard_thruster(int thrust)
{
     tx_ctrl_data[STAR_THRUST_LSB] = thrust & 0x00FF;
     tx_ctrl_data[STAR_THRUST_MSB] = (thrust & 0xFF00) >> 8;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::send_control_command()
{
     char * packet;
     int bytes;

     // Generate Packet and grab reference to it
     packetizer_.set_flags(0x03);
     packetizer_.set_csr_addr(0x00);          
     packetizer_.set_data(tx_ctrl_data, TX_CTRL_SIZE);
     bytes = packetizer_.generate_packet(&packet);    
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);
     
     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }          
     } while(status == Packetizer::In_Progress);
     
     if (status == Packetizer::Success) {
          bytes = receiver_.get_payload(&packet);
          
          //for (int x = 0 ; x < bytes ; x++) {
          //     printf("%x ", (unsigned char)packet[x]);
          //}
          //printf("\n");
          //short temp = 0;
          //temp = ((short)(packet[HEADING_MSB]) << 8) | packet[HEADING_LSB]; 
          //heading_ = temp;
          //
          //temp = 0;
          //temp = ((short)(packet[PITCH_MSB]) << 8) | packet[PITCH_LSB]; 
          //pitch_ = temp;
          //
          //temp = 0;
          //temp = ((short)(packet[ROLL_MSB]) << 8) | packet[ROLL_LSB]; 
          //roll_ = temp;
                    
     } else {
          printf("Decode Error.\n");
     }
     return VideoRayComm::Success;
}

short VideoRayComm::swap_bytes(char *array, int msb, int lsb)
{
     short temp = 0;
     temp = ((unsigned short)(array[msb] & 0xFF) << 8) | (array[lsb] & 0xFF);
     return temp;
}

VideoRayComm::Status_t VideoRayComm::send_nav_data_command()
{
     char * packet;
     int bytes;

     //////////////////////
     // Tx Sensor Message
     //////////////////////
     // Generate Packet and grab reference to it
     //// Packet for CSR map read...
     //packetizer_.set_flags(0x94);
     //packetizer_.set_csr_addr(0x66);          
     //packetizer_.set_data(tx_ctrl_data, 0);
     
     // Navigation data vendor specific message...
     packetizer_.set_flags(0x5);
     packetizer_.set_csr_addr(0x0);          
     packetizer_.set_data(tx_ctrl_data, 0);

     bytes = packetizer_.generate_packet(&packet);
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);
     
     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }          
     } while(status == Packetizer::In_Progress);
     
     if (status == Packetizer::Success) {
          bytes = receiver_.get_payload(&packet);
          
          //for (int x = 0 ; x < bytes ; x++) {
          //     printf("%x ", (unsigned char)packet[x]);
          //}
          //printf("\n");
                   
          heading_ = swap_bytes(packet, HEADING_MSB, HEADING_LSB) / 10.0;
          pitch_ = swap_bytes(packet, PITCH_MSB, PITCH_LSB) / 10.0;
          roll_ = swap_bytes(packet, ROLL_MSB, ROLL_LSB) / 10.0;

          depth_ = swap_bytes(packet, DEPTH_MSB, DEPTH_LSB) / 10.0;

          yaw_accel_ = swap_bytes(packet, YAW_ACC_MSB, YAW_ACC_LSB) / 1000.0;
          pitch_accel_ = swap_bytes(packet, PITCH_ACC_MSB, PITCH_ACC_LSB) / 1000.0;
          roll_accel_ = swap_bytes(packet, ROLL_ACC_MSB, ROLL_ACC_LSB) / 1000.0;

          surge_accel_ = swap_bytes(packet, SURGE_ACC_MSB, SURGE_ACC_LSB) / 1000.0;
          sway_accel_ = swap_bytes(packet, SWAY_ACC_MSB, SWAY_ACC_LSB) / 1000.0;          
          heave_accel_ = swap_bytes(packet, HEAVE_ACC_MSB, HEAVE_ACC_LSB) / 1000.0;          

          rov_power_ = swap_bytes(packet,  ROV_PWR_MSB, ROV_PWR_LSB);
          
     } else {
          printf("Decode Error.\n");
     }
     return VideoRayComm::Success;
}

double VideoRayComm::heading()
{
     return heading_;
}

double VideoRayComm::depth()
{
     return depth_;
}


double VideoRayComm::roll()
{
     return roll_;
}

double VideoRayComm::pitch()
{
     return pitch_;
}

double VideoRayComm::water_temperature()
{
     return water_temperature_;
}

double VideoRayComm::internal_temperature()
{
     return internal_temperature_;
}

double VideoRayComm::water_ingress()
{
     return water_ingress_;
}

double VideoRayComm::yaw_accel()
{
     return yaw_accel_;
}

double VideoRayComm::pitch_accel()
{
     return pitch_accel_;
}

double VideoRayComm::roll_accel()
{
     return roll_accel_;
}

double VideoRayComm::surge_accel()
{
     return surge_accel_;
}

double VideoRayComm::sway_accel()
{
     return sway_accel_;
}

double VideoRayComm::heave_accel()
{
     return heave_accel_;
}

