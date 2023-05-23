/// @file     suction_server.hpp
/// @brief    吸引用アクションノード
/*
Copyright (c) 2014 TOYOTA MOTOR CORPORATION
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
#ifndef TMC_SUCTION_SUCTION_SERVER_HPP_
#define TMC_SUCTION_SUCTION_SERVER_HPP_

#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <actionlib/action_definition.h>
#include <actionlib/server/action_server.h>
#include <tmc_suction/SuctionControlAction.h>

/// 吸引用ノードのネームスペース
namespace tmc_suction {

class SuctionServer {
 public:
  typedef boost::shared_ptr<actionlib::ActionServer<SuctionControlAction> > ActionServerPtr;

  void Run();

  /// コンストラクタ
  /// @param [in] node ノードハンドル
  /// @param [in] name アクション名
  SuctionServer(const ros::NodeHandle& node, const std::string name);

  ~SuctionServer() {}

 private:
  ros::NodeHandle node_handle_;
  actionlib::ActionServer<SuctionControlAction>::GoalHandle goal_handle_;

  /// アクション開始時間[s]
  double start_time_;

  /// アクションのタイムアウト時間[s]
  double timeout_;

  /// アクションサーバー
  ActionServerPtr suction_control_action_server_;

  SuctionControlResult suction_control_result_;

  /// 吸引開始命令の発行
  ros::Publisher suction_on_pub_;

  /// 吸引センサの状態の購読
  ros::Subscriber pressure_sensor_sub_;

  /// @brief アクションのGoal用コールバック
  /// @param [in] goal_handle Goalのハンドラ
  void GoalCallback(actionlib::ActionServer<SuctionControlAction>::GoalHandle goal_handle);

  /// @brief 物が実際に吸引されているかどうかの状態を受け取るコールバック
  /// @param [in] pressure_sensor_on 吸引センサの状態
  void PressureSensorCallback(const std_msgs::Bool& pressure_sensor_on);
};

}  /// end of namespace tmc_suction

#endif  /// TMC_SUCTION_SUCTION_SERVER_HPP_
