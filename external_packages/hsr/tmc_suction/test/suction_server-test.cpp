/// @file     suction_server-test.cpp
/// @brief    吸引用アクションノードのテスト
/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <gtest/gtest.h>
#include <tmc_suction/suction_server.hpp>

namespace {
const double kTimeout = 10.0;
const double kRate = 10.0;
}  // unnamed namespace

namespace tmc_suction {

/// 吸引アクションノードをテストするクラス
class SuctionServerTest : public ::testing::Test {
 protected:
  SuctionServerTest() {
    ros::NodeHandle node;
    suction_server_goal_pub_ = node.advertise<tmc_suction::SuctionControlActionGoal>("/suction_control/goal", 1, false);
    pressure_sensor_on_pub_ = node.advertise<std_msgs::Bool>("/pressure_sensor_on", 1, false);
    suction_server_result_sub_ =
        node.subscribe("/suction_control/result", 1, &SuctionServerTest::SuctionResultCallback_, this);
    suction_on_sub_ = node.subscribe("/suction_on", 1, &SuctionServerTest::SuctionOnCallback_, this);
  }
  virtual ~SuctionServerTest() {}
  virtual void SetUp() {
    // 適切なトピックとの接続が確立されるまで待つ
    ASSERT_TRUE(WaitUntilConnectionEstablished(kTimeout));
  }

  bool WaitUntilConnectionEstablished(double max_duration) {
    ros::Rate rate(kRate);
    ros::Time start_time = ros::Time::now();
    while (((suction_on_sub_.getNumPublishers() == 0) || (suction_server_goal_pub_.getNumSubscribers() == 0)) &&
           (ros::Time::now() - start_time) < ros::Duration(max_duration)) {
      rate.sleep();
    }
    return ((ros::Time::now() - start_time) < ros::Duration(max_duration));
  }

  bool WaitForSuctionOn(const ros::Duration& timeout) {
    is_sub_suction_on_ = false;

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(kRate);
    while (!is_sub_suction_on_ && (ros::Time::now() - start_time) < timeout) {
      ros::spinOnce();
      rate.sleep();
    }
    if (!is_sub_suction_on_) {
      return false;
    }
    return true;
  }

  bool WaitForResult(const ros::Duration& timeout) {
    is_sub_action_result_ = false;

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(kRate);
    while (!is_sub_action_result_ && (ros::Time::now() - start_time) < timeout) {
      ros::spinOnce();
      rate.sleep();
    }
    if (!is_sub_action_result_) {
      return false;
    }
    return true;
  }

  bool WaitForMessages(const ros::Duration& timeout) {
    is_sub_suction_on_ = false;
    is_sub_action_result_ = false;

    ros::Time start_time = ros::Time::now();
    ros::Rate rate(kRate);
    while ((!is_sub_suction_on_ || !is_sub_action_result_) && (ros::Time::now() - start_time) < timeout) {
      ros::spinOnce();
      rate.sleep();
    }
    if (!is_sub_suction_on_ || !is_sub_action_result_) {
      return false;
    }
    return true;
  }

  void SuctionResultCallback_(const tmc_suction::SuctionControlActionResult& msg) {
    suction_result_msg_ = msg;
    is_sub_action_result_ = true;
  }

  void SuctionOnCallback_(const std_msgs::Bool& msg) {
    suction_on_msg_ = msg;
    is_sub_suction_on_ = true;
  }

  ros::Publisher pressure_sensor_on_pub_;
  ros::Publisher suction_server_goal_pub_;
  ros::Subscriber suction_server_result_sub_;
  ros::Subscriber suction_on_sub_;

  bool is_sub_suction_on_;
  bool is_sub_action_result_;

  std_msgs::Bool suction_on_msg_;
  tmc_suction::SuctionControlActionResult suction_result_msg_;
};

TEST_F(SuctionServerTest, StartActionSuccess) {
  // goalを発行
  tmc_suction::SuctionControlActionGoal goal_msg;
  goal_msg.goal.timeout = ros::Duration(kTimeout);
  goal_msg.goal.suction_on.data = true;
  suction_server_goal_pub_.publish(goal_msg);

  ASSERT_TRUE(WaitForSuctionOn(ros::Duration(kTimeout)));
  EXPECT_TRUE(suction_on_msg_.data);

  // pressure_sensor_onを発行
  std_msgs::Bool pressure_sensor_msg;
  pressure_sensor_msg.data = true;
  pressure_sensor_on_pub_.publish(pressure_sensor_msg);

  ASSERT_TRUE(WaitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ(actionlib_msgs::GoalStatus::SUCCEEDED, suction_result_msg_.status.status);
}

TEST_F(SuctionServerTest, StartActionTimeout) {
  // goalを発行
  tmc_suction::SuctionControlActionGoal goal_msg;
  goal_msg.goal.timeout = ros::Duration(kTimeout);
  goal_msg.goal.suction_on.data = true;
  suction_server_goal_pub_.publish(goal_msg);

  ASSERT_TRUE(WaitForSuctionOn(ros::Duration(kTimeout)));
  EXPECT_TRUE(suction_on_msg_.data);

  // pressure_sensor_onを発行
  std_msgs::Bool pressure_sensor_msg;
  pressure_sensor_msg.data = false;

  ros::Time start_time = ros::Time::now();
  ros::Rate rate(kRate);
  while ((ros::Time::now() - start_time) < ros::Duration(kTimeout)) {
    pressure_sensor_on_pub_.publish(pressure_sensor_msg);
    rate.sleep();
  }

  ASSERT_TRUE(WaitForResult(ros::Duration(kTimeout)));
  EXPECT_FALSE(suction_on_msg_.data);
  EXPECT_EQ(actionlib_msgs::GoalStatus::ABORTED, suction_result_msg_.status.status);
}

TEST_F(SuctionServerTest, StopActionSuccess) {
  // goalを発行
  tmc_suction::SuctionControlActionGoal goal_msg;
  goal_msg.goal.suction_on.data = false;
  suction_server_goal_pub_.publish(goal_msg);

  ASSERT_TRUE(WaitForMessages(ros::Duration(kTimeout)));
  EXPECT_FALSE(suction_on_msg_.data);
  EXPECT_EQ(actionlib_msgs::GoalStatus::SUCCEEDED, suction_result_msg_.status.status);
}

TEST_F(SuctionServerTest, StartActionPreempt) {
  // goalを発行
  tmc_suction::SuctionControlActionGoal goal_msg;
  goal_msg.goal.timeout = ros::Duration(kTimeout);
  goal_msg.goal.suction_on.data = true;
  suction_server_goal_pub_.publish(goal_msg);

  ASSERT_TRUE(WaitForSuctionOn(ros::Duration(kTimeout)));
  EXPECT_TRUE(suction_on_msg_.data);

  // もう一度goalを発行
  suction_server_goal_pub_.publish(goal_msg);
  ASSERT_TRUE(WaitForMessages(ros::Duration(kTimeout)));
  EXPECT_TRUE(suction_on_msg_.data);
  EXPECT_EQ(actionlib_msgs::GoalStatus::PREEMPTED, suction_result_msg_.status.status);

  // pressure_sensor_onを発行
  std_msgs::Bool pressure_sensor_msg;
  pressure_sensor_msg.data = true;
  pressure_sensor_on_pub_.publish(pressure_sensor_msg);

  ASSERT_TRUE(WaitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ(actionlib_msgs::GoalStatus::SUCCEEDED, suction_result_msg_.status.status);
}

TEST_F(SuctionServerTest, StartActionErrorValue) {
  // goalを発行
  tmc_suction::SuctionControlActionGoal goal_msg;
  goal_msg.goal.timeout = ros::Duration(-kTimeout);
  goal_msg.goal.suction_on.data = true;
  suction_server_goal_pub_.publish(goal_msg);

  ASSERT_TRUE(WaitForResult(ros::Duration(kTimeout)));
  EXPECT_EQ(actionlib_msgs::GoalStatus::REJECTED, suction_result_msg_.status.status);
}

}  // namespace tmc_suction

int main(int argc, char** argv) {
  ros::init(argc, argv, "suction_server_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
