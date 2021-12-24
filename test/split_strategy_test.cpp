
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/split_strategy.hpp>

using namespace robosense::lidar;

TEST(TestSplitStrategyByAngle, newBlock)
{
  {
    SplitStrategyByAngle sa(10);
    ASSERT_FALSE(sa.newBlock(5));
    ASSERT_TRUE(sa.newBlock(15));
  }

  {
    SplitStrategyByAngle sa(10);
    ASSERT_FALSE(sa.newBlock(5));
    ASSERT_TRUE(sa.newBlock(10));
    ASSERT_FALSE(sa.newBlock(15));
  }

  {
    SplitStrategyByAngle sa(10);
    ASSERT_FALSE(sa.newBlock(10));
    ASSERT_FALSE(sa.newBlock(15));
  }
}

TEST(TestSplitStrategyByAngle, newBlock_Zero)
{
  {
    SplitStrategyByAngle sa(0);
    ASSERT_FALSE(sa.newBlock(35999));
    ASSERT_TRUE(sa.newBlock(1));
    ASSERT_FALSE(sa.newBlock(2));
  }

  {
    SplitStrategyByAngle sa(0);
    ASSERT_FALSE(sa.newBlock(35999));
    ASSERT_TRUE(sa.newBlock(0));
    ASSERT_FALSE(sa.newBlock(2));
  }

  {
    SplitStrategyByAngle sa(0);
    ASSERT_FALSE(sa.newBlock(0));
    ASSERT_FALSE(sa.newBlock(2));
  }
}

TEST(TestSplitStrategyByNum, newBlock)
{
  uint16_t max_blks = 2;
  SplitStrategyByNum sn(&max_blks);
  ASSERT_FALSE(sn.newBlock(0));
  ASSERT_TRUE(sn.newBlock(0));
  ASSERT_FALSE(sn.newBlock(0));
  ASSERT_TRUE(sn.newBlock(0));

  max_blks = 3;
  ASSERT_FALSE(sn.newBlock(0));
  ASSERT_FALSE(sn.newBlock(0));
  ASSERT_TRUE(sn.newBlock(0));
}

TEST(TestSplitStrategyBySeq, newPacket_max_seq)
{
  uint16_t max_seq = 2;
  SplitStrategyBySeq sn(&max_seq);

  // reach max_seq
  ASSERT_FALSE(sn.newPacket(1));
  ASSERT_TRUE(sn.newPacket(2));

  // reach max_seq again
  ASSERT_FALSE(sn.newPacket(1));
  ASSERT_TRUE(sn.newPacket(2));
}

TEST(TestSplitStrategyBySeq, newPacket_rewind)
{
  uint16_t max_seq = 3;
  SplitStrategyBySeq sn(&max_seq);

  // rewind
  ASSERT_FALSE(sn.newPacket(1));
  ASSERT_FALSE(sn.newPacket(2));
  ASSERT_TRUE(sn.newPacket(1));
}

