/// @file     suction_server.hpp
/// @brief    吸引用アクションノード
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
#include "tmc_suction/suction_server.hpp"
#include <string>

namespace tmc_suction {

SuctionServer::SuctionServer(const ros::NodeHandle& node, std::string name)
    : node_handle_(node), start_time_(0.0), timeout_(0.0) {
  suction_control_action_server_.reset(new actionlib::ActionServer<SuctionControlAction>(
      node_handle_, name, boost::bind(&SuctionServer::GoalCallback, this, _1), false));
  suction_control_action_server_->start();

  // SubscriberとPublisherの登録
  pressure_sensor_sub_ = node_handle_.subscribe("pressure_sensor_on", 1, &SuctionServer::PressureSensorCallback, this);
  suction_on_pub_ = node_handle_.advertise<std_msgs::Bool>("suction_on", 1);
}

void SuctionServer::Run() { ros::spin(); }

void SuctionServer::GoalCallback(actionlib::ActionServer<SuctionControlAction>::GoalHandle goal_handle) {
  std_msgs::Bool suction_on;

  // 既にアクション動作中の場合,前のアクションを停止
  if (goal_handle_.getGoal() && (goal_handle_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
    goal_handle_.setCanceled(suction_control_result_);
  }
  // 吸引開始アクション
  if (goal_handle.getGoal()->suction_on.data) {
    // 開始時間とタイムアウト時間の設定
    start_time_ = ros::Time::now().toSec();
    timeout_ = goal_handle.getGoal()->timeout.toSec();
    // durationの負値が来たらリジェクト
    if (goal_handle.getGoal()->timeout.toSec() < 0.0) {
      goal_handle.setRejected(suction_control_result_);
      return;
    }
    goal_handle.setAccepted();
    goal_handle_ = goal_handle;

    // 吸引開始
    suction_on.data = true;
    suction_on_pub_.publish(suction_on);
  } else {
    goal_handle.setAccepted();
    // 吸引終了アクション
    suction_on.data = false;
    suction_on_pub_.publish(suction_on);
    // Result(actionlib_msgs::GoalStatus::SUCCEEDED)を返す
    goal_handle.setSucceeded(suction_control_result_);
  }
}

void SuctionServer::PressureSensorCallback(const std_msgs::Bool& pressure_sensor_on) {
  // アクションが始まっていなければ終了
  if (!goal_handle_.getGoal() || (goal_handle_.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)) {
    return;
  }
  // 物を実際に吸引し始めたらResult(actionlib_msgs::GoalStatus::SUCCEEDED)を返す
  if (pressure_sensor_on.data) {
    goal_handle_.setSucceeded(suction_control_result_);
    return;
  }

  // タイムアウト時間を過ぎたらResult(actionlib_msgs::GoalStatus::ABORTED)を返す
  double current_time = ros::Time::now().toSec();
  if (current_time - start_time_ > timeout_) {
    goal_handle_.setAborted(suction_control_result_);
    // 吸引終了
    std_msgs::Bool msg;
    msg.data = false;
    suction_on_pub_.publish(msg);
  }
}

}  // namespace tmc_suction

int main(int argc, char* argv[]) {
  try {
    ros::init(argc, argv, "suction_server");
    ros::NodeHandle node;
    tmc_suction::SuctionServer suction_server(node, "suction_control");
    suction_server.Run();
  } catch (const std::exception& ex) {
    ROS_FATAL("Suction_server node cannot launch");
    exit(EXIT_FAILURE);
  }
  return EXIT_SUCCESS;
}
