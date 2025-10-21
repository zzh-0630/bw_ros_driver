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
 *  Copyright (C) 2025, BEWIS SENSING. All rights reserved.
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
 * 解析数据样本
 *  - 欧拉角 P/R/Y 
 *  - 加速度
 *  - 角速度
 *  - 磁场 
 *  - 四元数
 */
struct DataSample
{
  bool has_euler=false, has_acc=false, has_gyro=false, has_mag=false, has_quat=false;
  double P=0,R=0,Y=0;                 
  double ax_g=0, ay_g=0, az_g=0;      
  double gx_dps=0, gy_dps=0, gz_dps=0;
  double mx=0, my=0, mz=0;            
  double q0=1, q1=0, q2=0, q3=0;      
};

/*!
 * \brief 解析 bwsensing 协议
 *
 * 数据帧格式（自动输出模式）
 *   0x77 | LEN | ADDR | CMD | DATA... | CHK
 *   校验和从 LEN 开始，到 DATA 结束
 */
class BwsParser
{
private:

  //---------------------------------------------------------------------//
  //- 私有变量                                                           -//
  //---------------------------------------------------------------------//

  enum State { WAIT_77, READ_LEN, READ_PAYLOAD };
  State st_ = WAIT_77;
  std::vector<uint8_t> buf_{}; 
  uint8_t need_ = 0;                              //长度
  uint64_t ok_cnt_ = 0, bad_cnt_ = 0;

  //---------------------------------------------------------------------//
  //- 私有函数                                                           -//
  //---------------------------------------------------------------------//

  bool onFrame(DataSample& out);

public:

  //---------------------------------------------------------------------//
  //- 构造函数                                                           -//
  //---------------------------------------------------------------------//
  
  BwsParser();

  void reset();

  /*!
   * \brief 输入数据进行解析
   * \param[in] data 输入数据缓冲区
   * \param[in] n 输入数据长度
   * \param[out] out 解析后输出的数据样本列表
   * \return 成功解析的数据样本数量
   */
  size_t feed(const uint8_t* data, size_t n, std::vector<DataSample>& out);

  uint64_t ok()  const { return ok_cnt_; }
  
  uint64_t bad() const { return bad_cnt_; }
};

}