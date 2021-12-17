
#include <gtest/gtest.h>

#include <rs_driver/driver/decoder/split_strategy.hpp>

using namespace robosense::lidar;

TEST(TestSplitAngle, toSplit)
{
  {
    SplitAngle sa(10);
    ASSERT_FALSE(sa.toSplit(5));
    ASSERT_TRUE(sa.toSplit(15));
  }

  {
    SplitAngle sa(10);
    ASSERT_FALSE(sa.toSplit(5));
    ASSERT_TRUE(sa.toSplit(10));
    ASSERT_FALSE(sa.toSplit(15));
  }

  {
    SplitAngle sa(10);
    ASSERT_FALSE(sa.toSplit(10));
    ASSERT_FALSE(sa.toSplit(15));
  }
}

TEST(TestSplitAngle, toSplit_Zero)
{
  {
    SplitAngle sa(0);
    ASSERT_FALSE(sa.toSplit(35999));
    ASSERT_TRUE(sa.toSplit(1));
    ASSERT_FALSE(sa.toSplit(2));
  }

  {
    SplitAngle sa(0);
    ASSERT_FALSE(sa.toSplit(35999));
    ASSERT_TRUE(sa.toSplit(0));
    ASSERT_FALSE(sa.toSplit(2));
  }

  {
    SplitAngle sa(0);
    ASSERT_FALSE(sa.toSplit(0));
    ASSERT_FALSE(sa.toSplit(2));
  }
}

