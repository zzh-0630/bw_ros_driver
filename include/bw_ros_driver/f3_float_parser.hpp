#pragma once
#include <cstdint>
#include <vector>

#include "bw_ros_driver/bws_parser.hpp"

namespace bw 
{
/**
 * @brief F3 协议解析器
 *
 * 帧格式：
 *   [0]  Head          (0xF3)          
 *   [1]  CmdID         (默认 0x44)
 *   [2]  DataLen       (默认 0x3C = 60)
 *   [3]  Count         (自增计数)
 *   [4]  CRC8          (多为对 [Head,CmdID,Len,Count] 计算得到)
 *   [5..] Data         (float32 小端序：见文档)
 *   [end] CheckSum     (对 [CmdID .. Data最后一字节] 求和的 8bit)
 * 
 */
class F3FloatParser 
{
private:
  enum State 
  {
    WAIT_F3,
    READ_FIXED,  // 读取 CmdID/Len/Count/CRC8 共 4 字节
    READ_DATA,   // 读取 DataLen 字节数据
    READ_SUM     // 读取末尾校验和
  };

  // 状态
  State state_ = WAIT_F3;

  // 帧各部分缓存
  uint8_t cmd_ = 0;
  uint8_t len_ = 0;
  uint8_t cnt_ = 0;
  uint8_t crc8_ = 0;

  std::vector<uint8_t> data_;  // Data 段
  uint8_t sum_byte_ = 0;       // 末尾校验和

  // 统计
  uint64_t ok_cnt_ = 0;
  uint64_t bad_cnt_ = 0;

  // 内部工具
  static uint8_t crc8Calc(uint8_t init, const uint8_t* p, uint32_t len);
  static uint8_t sum8Calc(const uint8_t* p, size_t n);
  static float leF32(const uint8_t* p);

  bool onFrame(DataSample& out);

public:
  F3FloatParser() = default;

  /// 重置状态机
  void reset();

  /// 喂入若干字节；可能吐出 0..N 个 ImuSample
  size_t feed(const uint8_t* data, size_t n, std::vector<DataSample>& out);

  /// 统计
  uint64_t ok() const { return ok_cnt_; }
  uint64_t bad() const { return bad_cnt_; }
};
}