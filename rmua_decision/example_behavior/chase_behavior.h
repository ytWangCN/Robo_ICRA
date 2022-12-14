#ifndef ROBORTS_DECISION_CHASE_BEHAVIOR_H
#define ROBORTS_DECISION_CHASE_BEHAVIOR_H

#include "io/io.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "line_iterator.h"

namespace roborts_decision {
class ChaseBehavior {
 public:
  ChaseBehavior(ChassisExecutor* &chassis_executor,
                Blackboard* &blackboard,
                const std::string & proto_file_path) : chassis_executor_(chassis_executor),
                                                       blackboard_(blackboard),
                                                       proto_file_path_(proto_file_path) {


    chase_goal_.header.frame_id = "map";
    chase_goal_.pose.orientation.x = 0;
    chase_goal_.pose.orientation.y = 0;
    chase_goal_.pose.orientation.z = 0;
    chase_goal_.pose.orientation.w = 1;

    chase_goal_.pose.position.x = 0;
    chase_goal_.pose.position.y = 0;
    chase_goal_.pose.position.z = 0;

    chase_buffer_.resize(2);
    chase_count_ = 0;

    cancel_goal_ = true;
    roborts_common::ReadProtoFromTextFile(proto_file_path_, &decision_config);

  }

  void Run() {

    auto executor_state = Update();

    auto robot_map_pose = blackboard_->GetRobotMapPose();
    if (executor_state != BehaviorState::RUNNING) {
      if (!decision_config.master() && !get_buff_)
        this->GetBulletBuff();
      
      time_ = blackboard_->GetTime();
      if (!decision_config.master()) {
        if (time_ < buff_change_time_) {
          blackboard_->GetZone();
          this->GetBulletBuff();
          buff_change_time_ -= 60;
        }
      }

      if (!standed_) {
        this->Stand();
        ROS_INFO("standed!");
      }
      else {
        //this->ChaseWithCamera(robot_map_pose);
        this->ChaseWithRadar(robot_map_pose);
        if (delay_ = 5)
          standed_ = false;
      }
    }
  }

  void GetBulletBuff() {
    geometry_msgs::PoseStamped search_point;
    search_point.header.frame_id = "map";
    search_point.pose.position.x = blackboard_->GetZoneX(decision_config.blue());
    search_point.pose.position.y = blackboard_->GetZoneY(decision_config.blue());
    search_point.pose.position.z = 0.0;

    auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                                                              0.0,
                                                              0.0);
    search_point.pose.orientation = quaternion;
    get_buff_ = true;
    chassis_executor_->Execute(search_point);
  }

  void ChaseWithRadar(const geometry_msgs::PoseStamped robot_map_pose) {
    if (decision_config.blue()) {
      roborts_msgs::CarCoordinate red_coord = blackboard_->GetEnemyPose();

      geometry_msgs::PoseStamped enemy_goal;
      enemy_goal.header.frame_id = "map";
      enemy_goal.header.stamp = ros::Time::now();
      if (red_coord.red1x != -1) {
        enemy_goal.pose.position.x = red_coord.red1x;
        enemy_goal.pose.position.y = red_coord.red1y;
      }

      else if (red_coord.red2x != -1) {
        enemy_goal.pose.position.x = red_coord.red2x;
        enemy_goal.pose.position.y = red_coord.red2y;
      }
      else {
        delay_ += 1;
        return;
      }

      //filter
      if (!chase_init_) 
        delay_ = 0;
        this->SetGoal(enemy_goal);

      if (chase_init_ && blackboard_ ->GetDistance(chase_goal_, enemy_goal) > 0) {
        this->SetGoal(enemy_goal);

        auto robot_x = robot_map_pose.pose.position.x;
        auto robot_y = robot_map_pose.pose.position.y;
        unsigned int robot_cell_x, robot_cell_y;
        double goal_x, goal_y;
        blackboard_->GetCostMap2D()->World2Map(robot_x,
                                              robot_y,
                                              robot_cell_x,
                                              robot_cell_y);

          bool find_goal = false;
          for(FastLineIterator line(  red_coord.red1x, red_coord.red1x, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance()) {

            auto point_cost = blackboard_->GetCostMap2D()->GetCost((unsigned int) (line.GetX()), (unsigned int) (line.GetY())); //current point's cost

            if(point_cost >= 253)
              continue;
            chassis_executor_->Execute(enemy_goal);
          }
      }
    }
  }

  void ChaseWithCamera(const geometry_msgs::PoseStamped robot_map_pose) {
    chase_buffer_[chase_count_++ % 2] = blackboard_->GetEnemy();
    chase_count_ = chase_count_ % 2;

    auto dx = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - robot_map_pose.pose.position.x;
    auto dy = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - robot_map_pose.pose.position.y;
    auto yaw = std::atan2(dy, dx);

    if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 1.0 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.0) {
      if (cancel_goal_) {
        chassis_executor_->Cancel();
        cancel_goal_ = false;
      }
      return;

    } else {

      auto orientation = tf::createQuaternionMsgFromYaw(yaw);
      geometry_msgs::PoseStamped reduce_goal;
      reduce_goal.pose.orientation = robot_map_pose.pose.orientation;

      reduce_goal.header.frame_id = "map";
      reduce_goal.header.stamp = ros::Time::now();
      reduce_goal.pose.position.x = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.x - 1.2 * cos(yaw);
      reduce_goal.pose.position.y = chase_buffer_[(chase_count_ + 2 - 1) % 2].pose.position.y - 1.2 * sin(yaw);
      auto enemy_x = reduce_goal.pose.position.x;
      auto enemy_y = reduce_goal.pose.position.y;
      reduce_goal.pose.position.z = 1;
      unsigned int goal_cell_x, goal_cell_y;

      // if necessary add mutex lock
      //blackboard_->GetCostMap2D()->GetMutex()->lock();
      auto get_enemy_cell = blackboard_->GetCostMap2D()->World2Map(enemy_x,
                                                                  enemy_y,
                                                                  goal_cell_x,
                                                                  goal_cell_y);
      //blackboard_->GetCostMap2D()->GetMutex()->unlock();

      if (!get_enemy_cell) {
        
        return;
      }

      auto robot_x = robot_map_pose.pose.position.x;
      auto robot_y = robot_map_pose.pose.position.y;
      unsigned int robot_cell_x, robot_cell_y;
      double goal_x, goal_y;
      blackboard_->GetCostMap2D()->World2Map(robot_x,
                                            robot_y,
                                            robot_cell_x,
                                            robot_cell_y);

      if (true) {

        bool find_goal = false;
        for(FastLineIterator line( goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_x); line.IsValid(); line.Advance()) {

          auto point_cost = blackboard_->GetCostMap2D()->GetCost((unsigned int) (line.GetX()), (unsigned int) (line.GetY())); //current point's cost

          if(point_cost >= 253){
            continue;

          } else {
            find_goal = true;
            blackboard_->GetCostMap2D()->Map2World((unsigned int) (line.GetX()),
                                                    (unsigned int) (line.GetY()),
                                                    goal_x,
                                                    goal_y);

            reduce_goal.pose.position.x = goal_x;
            reduce_goal.pose.position.y = goal_y;
            break;
          }

        }
        if (find_goal) {
          cancel_goal_ = true;
          chassis_executor_->Execute(reduce_goal);
        } else {
          if (cancel_goal_) {
            chassis_executor_->Cancel();
            standed_ = false;
            cancel_goal_ = false;
          }
          return;
        }

      } else {
        cancel_goal_ = true;
        chassis_executor_->Execute(reduce_goal);
      }
    }
  }

  void Stand() {
    if (decision_config.blue()) {
      if (decision_config.master()) {
        geometry_msgs::PoseStamped last_position;
        last_position.header.frame_id = "map";
        last_position.pose.position.x = decision_config.blue_master_bot().stand_position().x();;
        last_position.pose.position.y = decision_config.blue_master_bot().stand_position().y();;
        last_position.pose.position.z = decision_config.blue_master_bot().stand_position().z();;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.blue_master_bot().stand_position().roll(),
                                                                  decision_config.blue_master_bot().stand_position().pitch(),
                                                                  decision_config.blue_master_bot().stand_position().yaw());
        last_position.pose.orientation = quaternion;
        
        chassis_executor_->Execute(last_position);
        standed_ = false;
      }
      if (!decision_config.master()) {
        geometry_msgs::PoseStamped last_position;
        last_position.header.frame_id = "map";
        last_position.pose.position.x = decision_config.blue_wing_bot().stand_position().x();;
        last_position.pose.position.y = decision_config.blue_wing_bot().stand_position().y();;
        last_position.pose.position.z = decision_config.blue_wing_bot().stand_position().z();;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.blue_wing_bot().stand_position().roll(),
                                                                  decision_config.blue_wing_bot().stand_position().pitch(),
                                                                  decision_config.blue_wing_bot().stand_position().yaw());
        last_position.pose.orientation = quaternion;
        
        chassis_executor_->Execute(last_position);
        standed_ = false;
      }
    }

    else {
      if (decision_config.master()) {
        geometry_msgs::PoseStamped last_position;
        last_position.header.frame_id = "map";
        last_position.pose.position.x = decision_config.red_master_bot().stand_position().x();;
        last_position.pose.position.y = decision_config.red_master_bot().stand_position().y();;
        last_position.pose.position.z = decision_config.red_master_bot().stand_position().z();;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.red_master_bot().stand_position().roll(),
                                                                  decision_config.red_master_bot().stand_position().pitch(),
                                                                  decision_config.red_master_bot().stand_position().yaw());
        last_position.pose.orientation = quaternion;
        
        chassis_executor_->Execute(last_position);
        standed_ = false;
      }
      if (!decision_config.master()) {
        geometry_msgs::PoseStamped last_position;
        last_position.header.frame_id = "map";
        last_position.pose.position.x = decision_config.red_wing_bot().start_position().x();;
        last_position.pose.position.y = decision_config.red_wing_bot().start_position().y();;
        last_position.pose.position.z = decision_config.red_wing_bot().start_position().z();;

        auto quaternion = tf::createQuaternionMsgFromRollPitchYaw(decision_config.red_wing_bot().start_position().roll(),
                                                                  decision_config.red_wing_bot().start_position().pitch(),
                                                                  decision_config.red_wing_bot().start_position().yaw());
        last_position.pose.orientation = quaternion;
        
        chassis_executor_->Execute(last_position);
        standed_ = false;
      }
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  void SetGoal(geometry_msgs::PoseStamped chase_goal) {
    chase_goal_ = chase_goal;
  }

  ~ChaseBehavior() = default;

 private:
  //! executor
  ChassisExecutor* const chassis_executor_;
  roborts_decision::DecisionConfig decision_config;
  const std::string & proto_file_path_;

  //! perception information
  Blackboard* const blackboard_;

  //! chase goal
  geometry_msgs::PoseStamped chase_goal_;

  //! chase buffer
  std::vector<geometry_msgs::PoseStamped> chase_buffer_;
  unsigned int chase_count_;

  //! cancel flag
  bool cancel_goal_;
  bool standed_ = false;
  bool chase_init_ = false;
  bool get_buff_ = false;
  int delay_ = 0;
  int time_;
  int buff_change_time_ = 120;
};
}

#endif //ROBORTS_DECISION_CHASE_BEHAVIOR_H
