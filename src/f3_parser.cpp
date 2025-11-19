/*!
 *  \file         f3_parser.cpp
 *  \brief        F3 协议解析实现
 */
#include "bw_ros_driver/f3_parser.hpp"

#include <cstring>  // std::memcpy

namespace bw {

// ------------------------ 小工具函数 ------------------------ //

static float readFloatLe(const uint8_t* p) {
  uint32_t u =
      static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
      (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}

uint8_t F3Parser::computeCrc8(const uint8_t* p, size_t len) {
  uint8_t crc = 0x00;         // CRC8_INITVALUE
  const uint8_t poly = 0x07;  // CRC8_POLY

  for (size_t i = 0; i < len; ++i) {
    crc ^= p[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x80u) {
        crc = static_cast<uint8_t>((crc << 1u) ^ poly);
      } else {
        crc <<= 1u;
      }
    }
  }
  return crc;
}

// ------------------------ 构造与重置 ------------------------ //

F3Parser::F3Parser()
    : state_(WAIT_HEAD),
      buf_(),
      expected_len_(0),
      ok_cnt_(0),
      bad_cnt_(0),
      crc_bad_cnt_(0) {}

void F3Parser::reset() {
  state_ = WAIT_HEAD;
  buf_.clear();
  expected_len_ = 0;
}

// ------------------------ 主状态机 ------------------------ //

size_t F3Parser::feed(const uint8_t* data, size_t n,
                      std::vector<F3Sample>& out) {
  size_t produced = 0;

  for (size_t i = 0; i < n; ++i) {
    uint8_t b = data[i];

    switch (state_) {
      case WAIT_HEAD: {
        if (b == 0xF3) {
          buf_.clear();
          buf_.push_back(b);  // Head
          state_ = READ_HEADER;
        }
        break;
      }

      case READ_HEADER: {
        buf_.push_back(b);  // 追加 CmdID 或 DataLen
        if (buf_.size() == 3) {
          // 累积到 [0]=Head, [1]=CmdID, [2]=DataLen
          uint8_t data_len = buf_[2];

          // DataLen 是 [DataLen..CheckSum] 的长度，因此整帧长度 = 2 + DataLen
          expected_len_ = static_cast<size_t>(2u + data_len);

          // 简单 sanity check：最小长度应该能容纳
          // Head+CmdID+DataLen+Count+CRC8+Data(至少几字节)+CheckSum
          if (expected_len_ < 10) {
            // 明显非法，重新寻帧
            ++bad_cnt_;
            state_ = WAIT_HEAD;
            buf_.clear();
          } else {
            state_ = READ_FRAME;
          }
        }
        break;
      }

      case READ_FRAME: {
        buf_.push_back(b);

        if (buf_.size() == expected_len_) {
          // 一帧收齐：buf_[0..expected_len_-1]
          const std::vector<uint8_t>& frame = buf_;

          bool frame_ok = false;

          if (frame.size() >=
              2 + 1 + 1 + 1 + 56 +
                  1)  // Head + CmdID + DataLen + Count + CRC8 + Data + CheckSum
          {
            // ---- 1) CRC8：从 Head(含) 到 Count(含) ----
            // 索引: [0]=Head, [1]=CmdID, [2]=DataLen, [3]=Count, [4]=CRC8
            uint8_t crc_calc = computeCrc8(&frame[0], 4);  // 0..3
            uint8_t crc_recv = frame[4];

            if (crc_calc == crc_recv) {
              // ---- 2) CheckSum：CmdID 到最后一个 Data 字节之和 ----
              // 这里沿用你之前的实现逻辑：sum(frame[1..last_data_index])
              // last_data_index = frame.size() - 2
              uint8_t sum = 0;
              for (size_t k = 1; k < frame.size() - 1; ++k) {
                // 按你的协议描述「CmdID 到 Data 最后一个字节」，
                // 这会把 CmdID / DataLen / Count / CRC8 / Data 都加进去
                sum = static_cast<uint8_t>(sum + frame[k]);
              }
              uint8_t checksum_recv = frame.back();

              if (sum == checksum_recv) {
                F3Sample sample;
                if (parseFrame(frame, sample)) {
                  ++ok_cnt_;
                  frame_ok = true;
                  out.push_back(sample);
                  ++produced;
                } else {
                  ++bad_cnt_;
                }
              } else {
                ++bad_cnt_;
              }
            } else {
              ++crc_bad_cnt_;
            }
          } else {
            ++bad_cnt_;
          }

          // 无论成功/失败，回到寻找下一帧
          state_ = WAIT_HEAD;
          buf_.clear();
          expected_len_ = 0;
        }
        break;
      }
    }  // switch
  }  // for

  return produced;
}

// ------------------------ 帧解析 ------------------------ //

bool F3Parser::parseFrame(const std::vector<uint8_t>& frame, F3Sample& s) {
  if (frame.size() < 2 + 1 + 1 + 1 + 56 + 1) {
    // 太短，连一个完整数据域都不够
    return false;
  }

  // 索引：
  // [0] Head(0xF3)
  // [1] CmdID
  // [2] DataLen
  // [3] Count
  // [4] CRC8
  // [5..60] Data (56 bytes)
  // [61] CheckSum (对于 DataLen=0x3C 的默认情况)
  s.cmd_id = frame[1];
  s.count = frame[3];

  const uint8_t* d = &frame[5];  // Data 起始

  s.pitch_deg = readFloatLe(d + 0);
  s.roll_deg = readFloatLe(d + 4);
  s.yaw_deg = readFloatLe(d + 8);

  s.acc_x_g = readFloatLe(d + 12);
  s.acc_y_g = readFloatLe(d + 16);
  s.acc_z_g = readFloatLe(d + 20);

  s.gyro_x_dps = readFloatLe(d + 24);
  s.gyro_y_dps = readFloatLe(d + 28);
  s.gyro_z_dps = readFloatLe(d + 32);

  s.mag_x = readFloatLe(d + 36);
  s.mag_y = readFloatLe(d + 40);
  s.mag_z = readFloatLe(d + 44);

  s.temperature_deg_c = readFloatLe(d + 48);
  // 微秒计数是 uint32_t（小端）
  {
    uint32_t tu = static_cast<uint32_t>(d[52]) |
                  (static_cast<uint32_t>(d[53]) << 8) |
                  (static_cast<uint32_t>(d[54]) << 16) |
                  (static_cast<uint32_t>(d[55]) << 24);
    s.time_us = tu;
  }

  s.valid = true;
  return true;
}

}  // namespace bw
