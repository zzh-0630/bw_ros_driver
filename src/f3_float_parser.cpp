#include "bw_ros_driver/f3_float_parser.hpp"

#include <algorithm>
#include <cstring>  

namespace bw {

// CRC8（poly=0x07，initValue=0x00）
uint8_t F3FloatParser::crc8Calc(uint8_t initValue, const uint8_t* pdata, uint32_t len) 
{
  uint8_t v = initValue;
  for (uint32_t i = 0; i < len; ++i) 
  {
    v ^= pdata[i];
    for (uint8_t j = 0; j < 8; ++j) 
    {
      if (v & 0x80u) 
      {
        v = static_cast<uint8_t>((v << 1u) ^ 0x07u);
      } else 
      {
        v <<= 1u;
      }
    }
  }
  return v;
}

// 简单 8bit 求和
uint8_t F3FloatParser::sum8Calc(const uint8_t* p, size_t n) 
{
  unsigned s = 0;
  for (size_t i = 0; i < n; ++i) 
    s += p[i];
  return static_cast<uint8_t>(s & 0xFFu);
}

float F3FloatParser::leF32(const uint8_t* p) 
{
  uint32_t u =
      static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
      (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  float f;
  std::memcpy(&f, &u, sizeof(float));
  return f;
}

void F3FloatParser::reset() 
{
  state_ = WAIT_F3;
  data_.clear();
}

size_t F3FloatParser::feed(const uint8_t* data, size_t n,
                      std::vector<DataSample>& out) 
{
  size_t produced = 0;

  for (size_t i = 0; i < n; ++i) 
  {
    uint8_t b = data[i];

    switch (state_) 
    {
      case WAIT_F3: 
      {
        if (b == 0xF3) 
          state_ = READ_FIXED;
        break;
      }

      case READ_FIXED: 
      {
        // 依次读入 CmdID, Len, Count, CRC8
        static int idx = 0;
        static uint8_t fixed[4];
        fixed[idx++] = b;
        if (idx == 4) 
        {
          cmd_ = fixed[0];
          len_ = fixed[1];
          cnt_ = fixed[2];
          crc8_ = fixed[3];

          // CRC8 验证（对 CmdID/Len/Count）
          const uint8_t head3[3] = {cmd_, len_, cnt_};
          uint8_t crc = crc8Calc(0x00u, head3, 3u);
          if (crc != crc8_) 
          {
            ++bad_cnt_;
            // 重新找同步
            state_ = WAIT_F3;
          } else 
          {
            data_.clear();
            data_.reserve(len_);
            state_ = READ_DATA;
          }
          idx = 0;
        }
        break;
      }

      case READ_DATA: 
      {
        data_.push_back(b);
        if (data_.size() == static_cast<size_t>(len_)) 
          state_ = READ_SUM;
        break;
      }

      case READ_SUM: 
      {
        sum_byte_ = b;

        // CheckSum = sum8([CmdID, Len, Count, CRC8, Data...])
        std::vector<uint8_t> sum_buf;
        sum_buf.reserve(4 + data_.size());
        sum_buf.push_back(cmd_);
        sum_buf.push_back(len_);
        sum_buf.push_back(cnt_);
        sum_buf.push_back(crc8_);
        sum_buf.insert(sum_buf.end(), data_.begin(), data_.end());

        uint8_t s = sum8Calc(sum_buf.data(), sum_buf.size());

        if (s == sum_byte_) 
        {
          DataSample s_out;
          if (onFrame(s_out)) 
          {
            out.push_back(s_out);
            ++produced;
          }
          ++ok_cnt_;
        } else 
        {
          ++bad_cnt_;
        }

        // 无论成功/失败，回到等待同步
        state_ = WAIT_F3;
        data_.clear();
        break;
      }
    }  // switch
  }

  return produced;
}

bool F3FloatParser::onFrame(DataSample& out) 
{
  // Data 至少包含到 offset 52 + 4 = 56 字节（温度 + 微秒计数）
  if (data_.size() < 56u)
  {
    return false;
  }

  // 布局（小端 float32）：
  //  0: X_pitch (deg)
  //  4: Y_roll  (deg)
  //  8: Z_yaw   (deg)
  // 12: AccX (g)       16: AccY (g)     20: AccZ (g)
  // 24: GyroX (deg/s)  28: GyroY        32: GyroZ
  // 36: MagX (gauss)   40: MagY         44: MagZ
  // 48: Temperature (°C)   52: usec (uint32)
  const uint8_t* d = data_.data();

  const float P_deg = leF32(&d[0]);
  const float R_deg = leF32(&d[4]);
  const float Y_deg = leF32(&d[8]);

  const float ax_g = leF32(&d[12]);
  const float ay_g = leF32(&d[16]);
  const float az_g = leF32(&d[20]);

  const float gx_dps = leF32(&d[24]);
  const float gy_dps = leF32(&d[28]);
  const float gz_dps = leF32(&d[32]);

  const float mx_gs = leF32(&d[36]);
  const float my_gs = leF32(&d[40]);
  const float mz_gs = leF32(&d[44]);

  // float temp_c  = leF32(&d[48]);  
  // uint32_t usec = static_cast<uint32_t>(d[52]) | ... 

  out.has_euler = true;
  out.P = static_cast<double>(P_deg);
  out.R = static_cast<double>(R_deg);
  out.Y = static_cast<double>(Y_deg);

  out.has_acc = true;
  out.ax_g = static_cast<double>(ax_g);
  out.ay_g = static_cast<double>(ay_g);
  out.az_g = static_cast<double>(az_g);

  out.has_gyro = true;
  out.gx_dps = static_cast<double>(gx_dps);
  out.gy_dps = static_cast<double>(gy_dps);
  out.gz_dps = static_cast<double>(gz_dps);

  out.has_mag = true;
  out.mx = static_cast<double>(mx_gs);
  out.my = static_cast<double>(my_gs);
  out.mz = static_cast<double>(mz_gs);

  // F3 数据不含四元数
  out.has_quat = false;

  return true;
}

}  // namespace bw
