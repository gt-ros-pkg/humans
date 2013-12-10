#ifndef PACKETIZER_H_
#define PACKETIZER_H_
/// ---------------------------------------------------------------------------
/// @file Packetizer.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-08-14 11:59:38 yellowin>
///
/// @version 1.0
/// Created: 13 Aug 2013
///
/// ---------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ----------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The Packetizer class ...
/// 
/// ----------------------------------------------------------------------------

class Packetizer {
private:
     unsigned char network_id_;
     unsigned char flags_;
     unsigned char csr_addr_;
     unsigned char length_;
     unsigned char header_chk_sum_;
     unsigned char total_chk_sum_;

     const unsigned char REQUEST_SYNC_MSB_;
     const unsigned char REQUEST_SYNC_LSB_;
     const unsigned char RESPONSE_SYNC_MSB_;
     const unsigned char RESPONSE_SYNC_LSB_;
     
     char *packet_;

     unsigned char generate_check_sum(char *buf, int length);

     int count_;
     int rx_bytes_;

protected:
public:

     enum Status_t
     {
          Success = 0,
          In_Progress = 1,
          Hdr_Chk_Sum_Err = 2,
          Total_Chk_Sum_Err = 3,
          Sync_Err = 4
     };

     Packetizer();

     void set_network_id(unsigned char network_id);
     void set_flags(unsigned char flags);
     void set_csr_addr(unsigned char csr_addr);     
     void set_data(char * data, int length);
     
     int generate_packet(char **packet);

     void reset();
     Packetizer::Status_t receive_packet(unsigned char byte);
     int get_payload(char ** packet);
};

#endif
