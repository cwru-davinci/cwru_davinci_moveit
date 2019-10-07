#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(DummyTest, basicTest){
EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}