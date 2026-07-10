
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/chan_angles.hpp>

using namespace robosense::lidar;

TEST(TestChanAngles, genUserChan)
{
  std::vector<int32_t> vert_angles;
  std::vector<uint16_t> user_chans;

  vert_angles.push_back(100);
  vert_angles.push_back(0);
  vert_angles.push_back(-100);
  vert_angles.push_back(200);

  ChanAngles::genUserChan (vert_angles, user_chans);
  ASSERT_EQ(user_chans.size(), 4);
  ASSERT_EQ(user_chans[0], 2);
  ASSERT_EQ(user_chans[1], 1);
  ASSERT_EQ(user_chans[2], 0);
  ASSERT_EQ(user_chans[3], 3);
}

TEST(TestChanAngles, loadFromFile)
{
  std::vector<int32_t> vert_angles, horiz_angles;

  // load
  ASSERT_EQ(ChanAngles::loadFromFile ("../test/res/angle.csv", 4, vert_angles, horiz_angles), 0);
  ASSERT_EQ(vert_angles.size(), 4);
  ASSERT_EQ(horiz_angles.size(), 4);
  ASSERT_EQ(vert_angles[0], 500);
  ASSERT_EQ(vert_angles[1], 250);
  ASSERT_EQ(vert_angles[2], 0);
  ASSERT_EQ(vert_angles[3], -250);

  ASSERT_EQ(horiz_angles[0], 10);
  ASSERT_EQ(horiz_angles[1], -20);
  ASSERT_EQ(horiz_angles[2], 0);
  ASSERT_EQ(horiz_angles[3], -100);

  // load again
  ASSERT_EQ(ChanAngles::loadFromFile ("../test/res/angle.csv", 4, vert_angles, horiz_angles), 0);
  ASSERT_EQ(vert_angles.size(), 4);
  ASSERT_EQ(horiz_angles.size(), 4);

  // load non-existing file
  ASSERT_LT(ChanAngles::loadFromFile ("../test/res/non_exist.csv", 4, vert_angles, horiz_angles), 0);
  ASSERT_EQ(vert_angles.size(), 0);
  ASSERT_EQ(horiz_angles.size(), 0);
}

TEST(TestChanAngles, loadFromDifop)
{
  uint8_t vert_angle_arr[] = {0x00, 0x01, 0x02, 
                              0x01, 0x03, 0x04,
                              0x01, 0x05, 0x06,
                              0x00, 0x07, 0x08};
  uint8_t horiz_angle_arr[] = {0x00, 0x01, 0x11,
                               0x01, 0x02, 0x22,
                               0x00, 0x03, 0x33,
                               0x01, 0x04, 0x44};

  std::vector<int32_t> vert_angles, horiz_angles;

  // load
  ASSERT_EQ(ChanAngles::loadFromDifop(
        (const RSCalibrationAngle*)vert_angle_arr, 
        (const RSCalibrationAngle*)horiz_angle_arr, 
        4,
        vert_angles, 
        horiz_angles), 0);

  ASSERT_EQ(vert_angles.size(), 4);
  ASSERT_EQ(horiz_angles.size(), 4);
  ASSERT_EQ(vert_angles[0], 258);
  ASSERT_EQ(vert_angles[1], -772);
  ASSERT_EQ(vert_angles[2], -1286);
  ASSERT_EQ(vert_angles[3], 1800);

  ASSERT_EQ(horiz_angles[0], 273);
  ASSERT_EQ(horiz_angles[1], -546);
  ASSERT_EQ(horiz_angles[2], 819);
  ASSERT_EQ(horiz_angles[3], -1092);

  // load again
  ASSERT_EQ(ChanAngles::loadFromDifop(
        (const RSCalibrationAngle*)vert_angle_arr, 
        (const RSCalibrationAngle*)horiz_angle_arr, 
        4,
        vert_angles, 
        horiz_angles), 0);
  ASSERT_EQ(vert_angles.size(), 4);
  ASSERT_EQ(horiz_angles.size(), 4);
}

TEST(TestChanAngles, memberLoadFromFile)
{
  ChanAngles angles(4);

  // not loading yet
  ASSERT_EQ(angles.chan_num_, 4);
  ASSERT_EQ(angles.vert_angles_.size(), 4);
  ASSERT_EQ(angles.horiz_angles_.size(), 4);
  ASSERT_EQ(angles.user_chans_.size(), 4);

  // load
  ASSERT_EQ(angles.loadFromFile ("../test/res/angle.csv"), 0);
  ASSERT_EQ(angles.user_chans_.size(), 4);
  ASSERT_EQ(angles.toUserChan(0), 3);
  ASSERT_EQ(angles.toUserChan(1), 2);
  ASSERT_EQ(angles.toUserChan(2), 1);
  ASSERT_EQ(angles.toUserChan(3), 0);
}

TEST(TestChanAngles, memberLoadFromFile_fail)
{
  ChanAngles angles(4);
  ASSERT_EQ(angles.chan_num_, 4);

  // load non-existing file
  ASSERT_LT(angles.loadFromFile ("../test/res/non_exist.csv"), 0);
  ASSERT_EQ(angles.vert_angles_.size(), 4);
  ASSERT_EQ(angles.vert_angles_[0], 0);
}

TEST(TestChanAngles, memberLoadFromDifop)
{
  uint8_t vert_angle_arr[] = {0x00, 0x01, 0x02, 
                              0x01, 0x03, 0x04,
                              0x01, 0x05, 0x06,
                              0x00, 0x07, 0x08};
  uint8_t horiz_angle_arr[] = {0x00, 0x01, 0x11,
                               0x01, 0x02, 0x22,
                               0x00, 0x03, 0x33,
                               0x01, 0x04, 0x44};

  ChanAngles angles(4);
  ASSERT_EQ(angles.chan_num_, 4);

  // load
  ASSERT_EQ(angles.loadFromDifop((const RSCalibrationAngle*)vert_angle_arr, (const RSCalibrationAngle*)horiz_angle_arr), 0);

  ASSERT_EQ(angles.vert_angles_.size(), 4);
  ASSERT_EQ(angles.vert_angles_[0], 258);
  ASSERT_EQ(angles.vert_angles_[1], -772);
  ASSERT_EQ(angles.vert_angles_[2], -1286);
  ASSERT_EQ(angles.vert_angles_[3], 1800);

  ASSERT_EQ(angles.horiz_angles_[0], 273);
  ASSERT_EQ(angles.horiz_angles_[1], -546);
  ASSERT_EQ(angles.horiz_angles_[2], 819);
  ASSERT_EQ(angles.horiz_angles_[3], -1092);

  ASSERT_EQ(angles.user_chans_.size(), 4);
  ASSERT_EQ(angles.toUserChan(0), 2);
  ASSERT_EQ(angles.toUserChan(1), 1);
  ASSERT_EQ(angles.toUserChan(2), 0);
  ASSERT_EQ(angles.toUserChan(3), 3);
}

TEST(TestChanAngles, memberLoadFromDifop_fail)
{
  uint8_t vert_angle_arr[] = {0x00, 0x01, 0x02, 
                              0x01, 0x03, 0x04,
                              0xFF, 0x05, 0x06,
                              0xFF, 0x07, 0x08};
  uint8_t horiz_angle_arr[] = {0x00, 0x11, 0x22,
                               0x01, 0x33, 0x44,
                               0xFF, 0x55, 0x66,
                               0xFF, 0x77, 0x88};

  ChanAngles angles(4);
  ASSERT_EQ(angles.chan_num_, 4);

  // load invalid difop
  ASSERT_LT(angles.loadFromDifop((const RSCalibrationAngle*)vert_angle_arr, (const RSCalibrationAngle*)horiz_angle_arr), 0);
  ASSERT_EQ(angles.vert_angles_.size(), 4);
  ASSERT_EQ(angles.vert_angles_[0], 0);
}

TEST(TestChanAngles, memberLoadFromDifop_fail_angle)
{
  uint8_t vert_angle_arr[] = {0x00, 0x01, 0x02, 
                              0x01, 0x03, 0x04,
                              0x01, 0x05, 0x06,
                              0x00, 0x07, 0x08};

  // -9000 <= angle < 18000
  ChanAngles angles(4);
  ASSERT_EQ(angles.chan_num_, 4);

  {
    // 9000
    uint8_t horiz_angle_arr[] = 
    {
      0x00, 0x01, 0x11,
      0x01, 0x02, 0x22,
      0x00, 0x03, 0x33,
      0x00, 0x46, 0x52   //18000
    };

    // load
    ASSERT_LT(angles.loadFromDifop((const RSCalibrationAngle*)vert_angle_arr, (const RSCalibrationAngle*)horiz_angle_arr), 0);
  }

  {
    // -9001
    uint8_t horiz_angle_arr[] = 
    {
      0x00, 0x01, 0x11,
      0x01, 0x02, 0x22,
      0x00, 0x03, 0x33,
      0x01, 0x23, 0x29
    };

    // load
    ASSERT_LT(angles.loadFromDifop((const RSCalibrationAngle*)vert_angle_arr, (const RSCalibrationAngle*)horiz_angle_arr), 0);
  }
}

// ================== loadFromDifopAIRYLITE tests ==================
// Buffer layout (102 bytes):
//   [0..47]   vertical values, big-endian uint16, 24 channels
//   [48..50]  vertical sign bits (3 bytes for 24 channels, LSB-first within each byte)
//   [51..98]  horizontal values, big-endian uint16, 24 channels
//   [99..101] horizontal sign bits
namespace
{
// Helper: build a 102-byte AIRYLITE angle buffer with all zeros
std::vector<uint8_t> makeAiryliteBuffer()
{
  return std::vector<uint8_t>(102, 0);
}

// Helper: set vertical value at channel i (big-endian)
void setVertValue(std::vector<uint8_t>& buf, int i, int32_t v)
{
  uint16_t uv = static_cast<uint16_t>(v < 0 ? -v : v);
  buf[0 + i * 2] = static_cast<uint8_t>((uv >> 8) & 0xFF);
  buf[0 + i * 2 + 1] = static_cast<uint8_t>(uv & 0xFF);
}

// Helper: set horizontal value at channel i (big-endian)
void setHorizValue(std::vector<uint8_t>& buf, int i, int32_t v)
{
  uint16_t uv = static_cast<uint16_t>(v < 0 ? -v : v);
  buf[51 + i * 2] = static_cast<uint8_t>((uv >> 8) & 0xFF);
  buf[51 + i * 2 + 1] = static_cast<uint8_t>(uv & 0xFF);
}

// Helper: set vertical sign bit at channel i
void setVertSign(std::vector<uint8_t>& buf, int i, bool sign)
{
  int byte_idx = 48 + i / 8;
  int bit_idx = i % 8;
  if (sign)
    buf[byte_idx] |= static_cast<uint8_t>(1 << bit_idx);
  else
    buf[byte_idx] &= static_cast<uint8_t>(~(1 << bit_idx));
}

// Helper: set horizontal sign bit at channel i
void setHorizSign(std::vector<uint8_t>& buf, int i, bool sign)
{
  int byte_idx = 99 + i / 8;
  int bit_idx = i % 8;
  if (sign)
    buf[byte_idx] |= static_cast<uint8_t>(1 << bit_idx);
  else
    buf[byte_idx] &= static_cast<uint8_t>(~(1 << bit_idx));
}
}  // namespace

TEST(TestChanAngles, loadFromDifopAIRYLITE)
{
  ChanAngles angles(24);

  // invalid input: null pointer and size too small
  ASSERT_LT(angles.loadFromDifopAIRYLITE(nullptr, 102), 0);
  std::vector<uint8_t> small_buf(101, 0);
  ASSERT_LT(angles.loadFromDifopAIRYLITE(small_buf.data(), small_buf.size()), 0);

  // normal parse: value decoding with sign bit, member state, user_chans
  auto buf = makeAiryliteBuffer();
  setVertValue(buf, 0, 400);  setVertSign(buf, 0, true);   // -400
  setVertValue(buf, 1, 200);                                // 200
  setHorizValue(buf, 10, 2500);                             // 2500
  setHorizValue(buf, 11, 1800); setHorizSign(buf, 11, true); // -1800
  ASSERT_EQ(angles.loadFromDifopAIRYLITE(buf.data(), buf.size()), 0);
  ASSERT_EQ(angles.vert_angles_[0], -400);
  ASSERT_EQ(angles.vert_angles_[1], 200);
  ASSERT_EQ(angles.horiz_angles_[10], 2500);
  ASSERT_EQ(angles.horiz_angles_[11], -1800);
  ASSERT_EQ(angles.user_chans_.size(), 24U);

  // range check: invalid vert 9000 (must be < 9000), state preserved
  auto buf_inv = makeAiryliteBuffer();
  setVertValue(buf_inv, 0, 9000);
  ASSERT_LT(angles.loadFromDifopAIRYLITE(buf_inv.data(), buf_inv.size()), 0);
  ASSERT_EQ(angles.vert_angles_[0], -400);   // unchanged from previous valid load
}

