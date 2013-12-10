#include <iostream>
#include <stdio.h>
#include <string.h>
#include "Packetizer.h"

#define SYNC_1     0
#define SYNC_2     1
#define NETWORK_ID 2
#define FLAGS      3
#define CSR_ADDR   4
#define LENGTH     5
#define HDR_SUM    6
#define PAYLOAD    7
// Total checksum is the last byte
#define TOTAL_SUM  8

#define HEADER_SIZE 7

Packetizer::Packetizer() : REQUEST_SYNC_MSB_(0xFA), REQUEST_SYNC_LSB_(0xAF), 
                           RESPONSE_SYNC_MSB_(0xFD), RESPONSE_SYNC_LSB_(0xDF) 
{
     count_ = 0;
     packet_ = NULL;
}

void Packetizer::set_network_id(unsigned char network_id)
{
     network_id_ = network_id;
}

void Packetizer::set_flags(unsigned char flags)
{
     flags_ = flags;
}

void Packetizer::set_csr_addr(unsigned char csr_addr)
{
     csr_addr_ = csr_addr;
}

void Packetizer::set_data(char * data, int length)
{
     length_ = length;
     
     if (packet_) {
          delete[] packet_;
     }

     // Packet consists of:
     // HEADER_SIZE (7 bytes)
     // Payload Data (length bytes)
     // Final Checksum (1 byte)
     packet_ = new char[HEADER_SIZE + length_ + 1];
     
     // copy data to internal buffer
     memcpy(packet_ + PAYLOAD, data, length_);     
}

unsigned char Packetizer::generate_check_sum(char *buf, int length)
{
     unsigned char result = 0;
     for (int i = 0; i < length; i++) {
          result ^= (unsigned char)buf[i];
     }
     return result;
}

int Packetizer::generate_packet(char ** packet)
{
     // Build packet     
     packet_[SYNC_1] = REQUEST_SYNC_MSB_;
     packet_[SYNC_2] = REQUEST_SYNC_LSB_;
     packet_[NETWORK_ID] = network_id_;
     packet_[FLAGS] = flags_;
     packet_[CSR_ADDR] = csr_addr_;
     packet_[LENGTH] = length_;
     packet_[HDR_SUM] = generate_check_sum(packet_, HEADER_SIZE-1);
     // Data has already been copied to payload
     packet_[HEADER_SIZE + length_] = generate_check_sum(packet_ + HEADER_SIZE, length_);

     *packet = packet_;
     
     return (HEADER_SIZE + length_ + 1);
}

void Packetizer::reset()
{
     count_ = 0;
}

Packetizer::Status_t Packetizer::receive_packet(unsigned char byte)
{
     bool in_prog = false;
     bool sync_err = false;

     switch (count_) {
     case SYNC_1:
          if (byte == RESPONSE_SYNC_MSB_) {
               count_++;
               in_prog = true;
          } else {
               sync_err = true;
          }
          break;

     case SYNC_2:
          if (byte == RESPONSE_SYNC_LSB_) {
               count_++;
               in_prog = true;
          } else {
               std::cout << "Sync Error" << std::endl;
               count_ = 0;
               sync_err = true;               
          }
          break;
          
     case NETWORK_ID:
          network_id_ = byte;
          count_++;
          in_prog = true;
          break;
          
     case FLAGS:
          flags_ = byte;
          count_++;
          in_prog = true;
          
          break;
     case CSR_ADDR:
          csr_addr_ = byte;
          count_++;
          in_prog = true;
          break;
          
     case LENGTH:
          length_ = byte;
          count_++;
          in_prog = true;
          
          if (packet_) {
               delete[] packet_;
          }
          packet_ = new char[length_];
          rx_bytes_ = 0;

          break;
     case HDR_SUM:
          header_chk_sum_ = byte;
          count_++;
          in_prog = true;

          break;
     case PAYLOAD:
          packet_[rx_bytes_++] = byte;          
          in_prog = true;
          if (rx_bytes_ >= length_) {
               count_++;
          }
          break;
          
     case TOTAL_SUM:
          total_chk_sum_ = byte;
          count_ = 0;
          return Packetizer::Success;

          break;

     default:
          std::cout << "Error in state machine" << std::endl;
     }

     if (sync_err) {
          return Packetizer::Sync_Err;
     }

     if (in_prog) {
          return Packetizer::In_Progress;
     }
}

int Packetizer::get_payload(char ** packet)
{
     *packet = packet_;
     return rx_bytes_;
}
