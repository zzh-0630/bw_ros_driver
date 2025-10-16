/*!
 *  \file         bws_parser.hpp
 *  \author       BW
 *  \date         15/10/2025
 *  \brief        Protocol parser of bwsensing devices.
 *
 *  Parser for the binary protocol of bwsensing devices.
 *
 *  \section CodeCopyright Copyright Notice
 *  MIT License
 *
 *  Copyright (c) 2025 BW
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#pragma once
#include <cstdint>
#include <vector>
#include "bw_ros_driver/bcd_utils.hpp"

namespace bw 
{

/*!
 * Parsed Data sample (units BEFORE ROS conversion):
 *  - Euler P/R/Y 
 *  - Angular rate
 *  - Acc
 *  - Mag 
 *  - Quaternion q0..q3 (w,x,y,z) 
 */
struct DataSample
{
  bool has_euler=false, has_acc=false, has_gyro=false, has_mag=false, has_quat=false;
  double P=0,R=0,Y=0;                 // deg
  double ax_g=0, ay_g=0, az_g=0;      // g
  double gx_dps=0, gy_dps=0, gz_dps=0;// deg/s
  double mx=0, my=0, mz=0;            // gauss/uT
  double q0=1, q1=0, q2=0, q3=0;      // w,x,y,z
};

/*!
 * \brief Streaming parser: feed bytes → 0..N DataSample(s)
 *
 * Frame format (auto-output):
 *   0x77 | LEN | ADDR | CMD | DATA... | CHK
 * Checksum here is sum8 over [LEN..DATA] 
 */
class BwsParser
{
private:

  //---------------------------------------------------------------------//
  //- Private variables                                                 -//
  //---------------------------------------------------------------------//

  enum State { WAIT_77, READ_LEN, READ_PAYLOAD };
  State st_ = WAIT_77;
  std::vector<uint8_t> buf_{}; 
  uint8_t need_ = 0;                              //length
  uint64_t ok_cnt_ = 0, bad_cnt_ = 0;

  //---------------------------------------------------------------------//
  //- Private  methods                                                  -//
  //---------------------------------------------------------------------//

  bool onFrame(DataSample& out);

public:

  //---------------------------------------------------------------------//
  //- Constructor                                                       -//
  //---------------------------------------------------------------------//
  
  BwsParser();

  void reset();

  /*!
   * \brief Feed a chunk of bytes. Push parsed samples into 'out'.
   * \return number of samples pushed
   */
  size_t feed(const uint8_t* data, size_t n, std::vector<DataSample>& out);

  uint64_t ok()  const { return ok_cnt_; }
  
  uint64_t bad() const { return bad_cnt_; }
};

} // namespace bw