
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


