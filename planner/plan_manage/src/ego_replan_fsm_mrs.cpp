

#include <plan_manage/ego_replan_fsm.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/TransformReferenceSrv.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/TransformPoseSrv.h>

namespace ego_planner
{
  EGOReplanFSM::~EGOReplanFSM()
  {
    result_file_.close();
  }
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_swarm_relative_pts_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    flag_relan_astar_ = false;
    have_local_traj_ = false;

    //nh.param("manager/drone_id", id, 0);
    //nh.param("manager/drone_leader", leader, string("uav1"));

    //nh.param<bool>("manager/name_list", name_list, false);
    nh.param<std::vector<std::string>>("manager/robot_names", robot_names, std::vector<std::string>());
    nh.param<std::string>("manager/robot_name", name_robot, std::string());
    nh.param<std::string>("manager/computer_name", name_computer, std::string());

    number_of_robots = robot_names.size();
    ROS_INFO("Number of Robots:  %i", number_of_robots);

    ROS_INFO("Name of Robot: %s", name_robot.c_str());
    for (size_t i = 0; i < robot_names.size(); ++i)
    {
      std::string robot_name_list = robot_names[i].c_str();

      if (i == 0)
      {
        leader = robot_name_list;
        ROS_INFO("Leader: %s", leader.c_str());
      }

      if (robot_name_list == name_robot)
      {
        id = i;
        ROS_INFO("Name of Robot: %s, Id: %i", robot_name_list.c_str(), id);
      }
    }
    nh.param<double>("uav_height", uav_height, 3.0);
    nh.param<double>("distance_target", distance_target, 0.5);
    nh.param<bool>("distance_type", distance_type, false);
    nh.param<bool>("wait_takeoff", wait_takeoff, true);
    nh.param<bool>("wait_odom", wait_odom, true);
    nh.param<bool>("have_swarm_relative_pts_", have_swarm_relative_pts_, false);

    nh.param<int>("path_number", path_number, -1);

    ROS_INFO("Height: %f", uav_height);
    ROS_INFO("Minimum stopping distance: %f", distance_target);

    leader_swarm = false;
    if (id == 0)
    {
      leader_swarm = true;
    }

    if (have_swarm_relative_pts_ == true)
    {
      ROS_INFO("[SWARM_FSM]: Have swarm relative points.");
      for (int i = 0; i < number_of_robots; i++)
      {
        nh.param("global_goal/relative_pos_" + to_string(i) + "/x", swarm_relative_pts_[i][0], -1.0);
        nh.param("global_goal/relative_pos_" + to_string(i) + "/y", swarm_relative_pts_[i][1], -1.0);
        nh.param("global_goal/relative_pos_" + to_string(i) + "/z", swarm_relative_pts_[i][2], -1.0);
      }
    }
    else
    {
      ROS_INFO("[SWARM_FSM]: Do not have swarm relative points. Waiting for the base computer.");
    }

    nh.param("global_goal/swarm_scale", swarm_scale_, 1.0);

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    // safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    // relative_pos_sub_ = nh.subscribe("positions", 10, &EGOReplanFSM::poseArrayCallback, this);
    relative_pos_sub_ = nh.subscribe("/" + name_computer + "/swarm_formation/positions", 10, &EGOReplanFSM::poseArrayCallback, this);

    // poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);

    central_goal = nh.subscribe("/" + leader + "/move_base_simple/swarm", 1, &EGOReplanFSM::formationWaypointCallback, this);
    central_goal_topic = nh.subscribe("/" + leader + "/move_base_path/swarm", 1, &EGOReplanFSM::formationCallback, this);

    // MRS System
    // // Set up the service client
    // client = nh.serviceClient<mrs_msgs::Vec4>("/uav" + to_string(id+1) + "/control_manager/goto");
    // client_octomap = nh.serviceClient<mrs_msgs::Vec4>("/uav" + to_string(id+1) + "/octomap_planner/goto");
    // client_octomap_stop = nh.serviceClient<std_srvs::Trigger>("/uav" + to_string(id+1) + "/octomap_planner/stop");

    // // Octomap Diagnostics
    // octoplanner_diagnostic_sub = nh.subscribe("/uav" + to_string(id+1) + "/octomap_planner/diagnostics", 1, &EGOReplanFSM::octoplannerDiagnosticsCallback, this);

    // Set up the service client
    client = nh.serviceClient<mrs_msgs::Vec4>("/"+ name_robot + "/control_manager/goto");
    client_octomap = nh.serviceClient<mrs_msgs::Vec4>("/"+ name_robot + "/octomap_planner/goto");
    client_octomap_stop = nh.serviceClient<std_srvs::Trigger>("/"+ name_robot + "/octomap_planner/stop");
    client_octomap_ref = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("/"+ name_robot + "/octomap_planner/reference");
    client_transform = nh.serviceClient<mrs_msgs::TransformPoseSrv>("/"+ name_robot + "/control_manager/transform_pose");
    client_leader_transform = nh.serviceClient<mrs_msgs::TransformPoseSrv>("/"+ leader + "/control_manager/transform_pose");

    // Octomap Diagnostics
    current_flag = false;
    octoplanner_diagnostic_sub = nh.subscribe("/"+ name_robot + "/octomap_planner/diagnostics", 1, &EGOReplanFSM::octoplannerDiagnosticsCallback, this);

    // Controller Diagnostics
    controller_diagnostic_sub = nh.subscribe("/"+ name_robot + "/control_manager/diagnostics", 1, &EGOReplanFSM::controllerDiagnosticsCallback, this);

    // Octomap Planner initial paths
    octoplanner_path = nh.subscribe("/" + leader + "/octomap_planner/path", 1, &EGOReplanFSM::octoplannerPathCallback, this);
    send_service_ = false;
    send_service_exe_ = false;
    receive_path_ = false;
    have_path_ = false;

    // uav_number = 3;
    // odom_sub2_ = nh.subscribe("/uav2/estimation_manager/odom_main", 1, &EGOReplanFSM::odometryCallback2, this);
    // odom_sub3_ = nh.subscribe("/uav3/estimation_manager/odom_main", 1, &EGOReplanFSM::odometryCallback3, this);

    dist_target = false;
    flying_normally_flag = false;
    end_pt_vector_size = 0;
    end_pt_vector_id = 0;

  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 1000)
    {
      fsm_num = 0;
      ROS_INFO("Execute FSM MRS Swarm.");
    }

    switch (exec_state_)
    {
    case INIT:
    {
      name_state = "FSM" + to_string(id);

      if (!have_odom_ && wait_odom)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_FORMATION, name_state);
      break;
    }

    case WAIT_FORMATION:
    {
      name_state = "FSM" + to_string(id);
      
      // std_srvs::Trigger srv;
      // client_octomap_stop.call(srv);

      if (!flying_normally_flag && wait_takeoff)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, name_state);
      break;
    }

    case WAIT_TARGET:
    {
      name_state = "FSM" + to_string(id);

      send_service_ = false;
      send_service_exe_ = false;
      receive_path_ = false;
      have_path_ = false;

      if (!have_swarm_relative_pts_)
      {
        ROS_WARN_ONCE("[SWARM_FSM]: Waiting for swarm relative points.");
        goto force_return; // return;
      }

      if (end_pt_vector_size > 0)
      {
        ROS_INFO("[SWARM_FSM]: Execute path: %d", end_pt_vector_id);
        end_pt_ = end_pt_vector_[end_pt_vector_id];
        have_target_ = true;
      }
      else
      {
        have_target_ = false;
      }

      ROS_WARN_ONCE("[SWARM_FSM]: Waiting for target.");
      if (!have_target_)
        goto force_return; // return;
      else
      {
        have_target_ = false;
        changeFSMExecState(SEQUENTIAL_START, name_state);
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      name_state = "FSM" + to_string(id);

      Eigen::Vector3d distance_target_;
      mrs_msgs::TransformPoseSrv srv;
      srv.request.pose.pose.position.x = float(end_pt_(0));
      srv.request.pose.pose.position.y = float(end_pt_(1));
      srv.request.pose.pose.position.z = float(end_pt_(2));
      srv.request.pose.header.stamp = ros::Time::now();
      srv.request.pose.header.frame_id = name_robot + "/utm_navsat"; // Assuming the frame is utm_navsat
      // ROS_INFO("[SWARM_FSM]: End point [%d]: x: %f, y: %f, z: %f", id, srv.request.pose.pose.position.x, srv.request.pose.pose.position.y, srv.request.pose.pose.position.z);
      srv.request.pose.pose.orientation.w = 1.0; // Assuming no rotation
      srv.request.frame_id = odom_frame_id_.c_str();
      // ROS_INFO("[SWARM_FSM]: Requesting transform for end point [%d] in frame %s", id, odom_frame_id_.c_str());

      if (client_transform.call(srv))
      {
        // ROS_INFO("[SWARM_FSM]: Transform service call succeeded");
        end_pt_tf_(0) = srv.response.pose.pose.position.x;
        end_pt_tf_(1) = srv.response.pose.pose.position.y;
        end_pt_tf_(2) = srv.response.pose.pose.position.z;
        // ROS_INFO("[SWARM_FSM]: End point TF [%d]: x: %f, y: %f, z: %f", id, srv.response.pose.pose.position.x, srv.response.pose.pose.position.y, srv.response.pose.pose.position.z);
      }
      else
      {
        ROS_ERROR("[SWARM_FSM]: Failed to call transform service");
      }

      // ROS_WARN_ONCE("[SWARM_FSM]: odometry position: x: %f, y: %f, z: %f", 
      //               float(odom_pos_(0)), float(odom_pos_(1)), float(odom_pos_(2)));
      // ROS_WARN_ONCE("[SWARM_FSM]: end point position: x: %f, y: %f, z: %f",
      //               float(end_pt_tf_(0)), float(end_pt_tf_(1)), float(end_pt_tf_(2)));

      // odom_pos_ is vector3d with x, y, z coordinates
      distance_target_ = odom_pos_ - end_pt_tf_;

      // Choose the type of distance calculation (false: norm of vector, true: xy norm)
      if (distance_type == false)
      {
        distance_to_target = distance_target_.norm();
      }
      else
      {
        distance_to_target = calculateXYNorm(distance_target_);
      }

      // Check that the distance is within tolerance
      if (distance_to_target < distance_target)
      {
        dist_target = true;
      }

      // Leader will check that it has reached the correct position
      if ((dist_target == true) && (leader_swarm == true))
      {
        dist_target = false;
        end_pt_vector_id ++;
        end_pt_vector_size--;
        changeFSMExecState(WAIT_TARGET, name_state);
        break;
      }

      // Leader will call octomap planner service
      if ((send_service_ == false) && (leader_swarm == true))
      {
        ROS_INFO("[SWARM_FSM]: Leader calls octomap planner service.");
        // mrs_msgs::Vec4 srv;
        // srv.request.goal[0] = float(end_pt_.x());
        // srv.request.goal[1] = float(end_pt_.y());
        // srv.request.goal[2] = float(end_pt_.z());
        // srv.request.goal[3] = 0.0;  // Assuming `w` is not used for position

        mrs_msgs::ReferenceStampedSrv srv;
        srv.request.header.frame_id = name_robot + "/utm_navsat";
        srv.request.reference.position.x = float(end_pt_.x());
        srv.request.reference.position.y = float(end_pt_.y());
        srv.request.reference.position.z = float(end_pt_.z());
        srv.request.reference.heading   = 0;

        if (client_octomap_ref.call(srv)) 
        {
          ROS_INFO("[SWARM_FSM]: Service call succeeded");
          if (srv.response.success == true)
          {
            ROS_INFO("[SWARM_FSM]: Planning.");
            send_service_ = true;
          }
          else
          {
            ROS_ERROR_STREAM("[SWARM_FSM]: " << srv.response.message);
            ROS_ERROR("[SWARM_FSM]: Send new target again.");
            end_pt_vector_.clear();
            end_pt_vector_size = end_pt_vector_.size();
            changeFSMExecState(WAIT_TARGET, name_state);
            break;
          }
        } 
        else 
        {
          ROS_ERROR("[SWARM_FSM]: Failed to call service.");
          changeFSMExecState(WAIT_TARGET, name_state);
          break;
        }
      }

      // Check if leader is stuck.
      static int stuck_checker = 0;
      stuck_checker++;
      if (stuck_checker == 1000)
      {
        stuck_checker = 0;
        if ((current_flag == true) && (leader_swarm == true))
        {
          ROS_ERROR("[SWARM_FSM]: Leader is stuck.");
          send_service_ = false;
        }
        else
        {
          ROS_INFO("[SWARM_FSM]: Leader is moving to %d", end_pt_vector_id);
          ROS_INFO("[SWARM_FSM]: Distance to target pose [%d]: %f", id, distance_to_target);
        }
      }

      // Follower is waiting for the leader
      static int follower_status = 0;
      follower_status++;
      if (follower_status == 1000)
      {
        follower_status = 0;
        if (leader_swarm == false)
        {
          ROS_INFO("[SWARM_FSM]: Waiting the leader.");
        }
      }

      if (!have_path_)
        goto force_return; // return;
      else
      {
        have_path_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, name_state);
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      name_state = "FSM" + to_string(id);
      bool success;

      if (leader_swarm == false)
      {
        int num_poses = get_path_.poses.size();
        //ROS_INFO("[SWARM_FSM]: Published path with %d poses.", num_poses);
        if (num_poses > 1)
        {
          geometry_msgs::PoseStamped last_pose;
          last_pose = get_path_.poses.back();

          mrs_msgs::TransformPoseSrv srv;
          srv.request.pose.pose.position.x = float(last_pose.pose.position.x);
          srv.request.pose.pose.position.y = float(last_pose.pose.position.y);
          srv.request.pose.pose.position.z = float(last_pose.pose.position.z);
          srv.request.pose.header.stamp = ros::Time::now();
          srv.request.pose.header.frame_id = leader + "/liosam_origin"; // Assuming the frame is liosam_origin
          ROS_INFO("[SWARM_FSM]: Last point [%d]: x: %f, y: %f, z: %f", id, srv.request.pose.pose.position.x, srv.request.pose.pose.position.y, srv.request.pose.pose.position.z);
          srv.request.pose.pose.orientation.w = 1.0; // Assuming no rotation
          srv.request.frame_id = leader + "/utm_navsat";

          ROS_INFO("[SWARM_FSM]: Requesting transform for last pose in frame %s", srv.request.frame_id.c_str());

          if (client_leader_transform.call(srv))
          {
            ROS_INFO("[SWARM_FSM]: Transform service call succeeded");
            ROS_INFO("[SWARM_FSM]: Transformed last pose: x: %f, y: %f, z: %f", 
                     float(srv.response.pose.pose.position.x), float(srv.response.pose.pose.position.y), float(srv.response.pose.pose.position.z));
            swarm_central_path_(0) = srv.response.pose.pose.position.x;
            swarm_central_path_(1) = srv.response.pose.pose.position.y;
            swarm_central_path_(2) = srv.response.pose.pose.position.z;
          }
          else
          {
            ROS_ERROR("[SWARM_FSM]: Failed to call transform service");
          }

          ROS_INFO("[SWARM_FSM]: Central path: x: %f, y: %f, z: %f", 
                   float(swarm_central_path_(0)), float(swarm_central_path_(1)), float(swarm_central_path_(2)));
          // swarm_central_path_(0) = last_pose.pose.position.x;
          // swarm_central_path_(1) = last_pose.pose.position.y;
          // swarm_central_path_(2) = last_pose.pose.position.z;

          Eigen::Vector3d relative_pos;
          relative_pos << swarm_relative_pts_[id][0],
                          swarm_relative_pts_[id][1],
                          swarm_relative_pts_[id][2];
          end_path_ = swarm_central_path_ + swarm_scale_ * relative_pos;

          ROS_INFO("[SWARM_FSM]: Final position [%d]: x: %f, y: %f, z: %f", id, float(end_path_.x()), float(end_path_.y()), float(end_path_.z()));
          success = true;
        }
        else
        {
          success = false;
        }
      }
      else
      {
        success = true;
      }

      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, name_state);
      }
      else
      {
        changeFSMExecState(WAIT_TARGET, name_state);
      }
      break;
    }

    case EXEC_TRAJ:
    {
      name_state = "FSM" + to_string(id);

      // Eigen::Vector3d distance_;
      // distance_ = odom_pos_ - end_path_;
      
      if ((send_service_exe_ == false) && (leader_swarm == false))
      {
        // ROS_INFO("[SWARM_FSM]: Call service partial target.");
        // mrs_msgs::Vec4 srv;
        // srv.request.goal[0] = float(end_path_.x());
        // srv.request.goal[1] = float(end_path_.y());
        // srv.request.goal[2] = float(end_path_.z());
        // srv.request.goal[3] = 0.0;  // Assuming `w` is not used for position

        mrs_msgs::ReferenceStampedSrv srv;
        srv.request.header.frame_id = name_robot + "/utm_navsat";
        srv.request.reference.position.x = float(end_path_.x());
        srv.request.reference.position.y = float(end_path_.y());
        srv.request.reference.position.z = float(end_path_.z());
        srv.request.reference.heading   = 0;

        if (client_octomap_ref.call(srv)) 
        // if (client_octomap.call(srv)) 
        {
          // ROS_INFO("[SWARM_FSM]: Service call succeeded");
          send_service_exe_ = true;
        } 
        else 
        {
          // ROS_ERROR("[SWARM_FSM]: Failed to call service");
          send_service_exe_ = false;
        }
      }

      if (leader_swarm == true)
      {
        send_service_exe_ = true;
      }

      if (!send_service_exe_)
        goto force_return; // return;
      else
      {
        have_target_ = false;
        send_service_ = false;
        send_service_exe_ = false;
        receive_path_ = false;
        have_path_ = false;
        changeFSMExecState(REPLAN_TRAJ, name_state);
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      name_state = "FSM" + to_string(id);

      if (false)
      {
        goto force_return; // return;
      }
      changeFSMExecState(SEQUENTIAL_START, name_state);
      break;
    }

    case EMERGENCY_STOP:
    {
      name_state = "FSM" + to_string(id);

      if (false)
      {
        goto force_return; // return;
      }
      changeFSMExecState(INIT, name_state);
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }


  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    odom_frame_id_ = msg->header.frame_id;

    have_odom_ = true;
  }


  void EGOReplanFSM::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
  {
    ROS_INFO("Received %lu poses", msg->poses.size());
    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
      const auto& p = msg->poses[i].position;
      if (i == 0)
      {
        swarm_central_pos_[0] = p.x;
        swarm_central_pos_[1] = p.y;
        swarm_central_pos_[2] = p.z;
      }
      swarm_relative_pts_[i][0] = p.x - swarm_central_pos_[0];
      swarm_relative_pts_[i][1] = p.y - swarm_central_pos_[1];
      swarm_relative_pts_[i][2] = p.z - swarm_central_pos_[2];
      ROS_INFO("Pose %lu: x=%.2f, y=%.2f, z=%.2f", i, swarm_relative_pts_[i][0], swarm_relative_pts_[i][1], swarm_relative_pts_[i][2]);
      
      Eigen::Vector3d relative_pos;
      relative_pos << swarm_relative_pts_[i][0],
                      swarm_relative_pts_[i][1],
                      swarm_relative_pts_[i][2];
      end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos;

      end_pt_vector_.push_back(end_pt_);
    }
    have_swarm_relative_pts_ = true;

    end_pt_vector_size = 1; // end_pt_vector_.size();
    // ROS_INFO("[SWARM_FSM]: Number of Poses: %d", end_pt_vector_size);
  }

  // void EGOReplanFSM::odometryCallback2(const nav_msgs::OdometryConstPtr &msg)
  // {
  //   odom_pos2_(0) = msg->pose.pose.position.x;
  //   odom_pos2_(1) = msg->pose.pose.position.y;
  //   odom_pos2_(2) = msg->pose.pose.position.z;

  //   odom_vel2_(0) = msg->twist.twist.linear.x;
  //   odom_vel2_(1) = msg->twist.twist.linear.y;
  //   odom_vel2_(2) = msg->twist.twist.linear.z;

  //   odom_orient_.w() = msg->pose.pose.orientation.w;
  //   odom_orient_.x() = msg->pose.pose.orientation.x;
  //   odom_orient_.y() = msg->pose.pose.orientation.y;
  //   odom_orient_.z() = msg->pose.pose.orientation.z;

  // }

  // void EGOReplanFSM::odometryCallback3(const nav_msgs::OdometryConstPtr &msg)
  // {
  //   odom_pos3_(0) = msg->pose.pose.position.x;
  //   odom_pos3_(1) = msg->pose.pose.position.y;
  //   odom_pos3_(2) = msg->pose.pose.position.z;

  //   odom_vel3_(0) = msg->twist.twist.linear.x;
  //   odom_vel3_(1) = msg->twist.twist.linear.y;
  //   odom_vel3_(2) = msg->twist.twist.linear.z;

  //   odom_orient_.w() = msg->pose.pose.orientation.w;
  //   odom_orient_.x() = msg->pose.pose.orientation.x;
  //   odom_orient_.y() = msg->pose.pose.orientation.y;
  //   odom_orient_.z() = msg->pose.pose.orientation.z;

  // }

  void EGOReplanFSM::octoplannerDiagnosticsCallback(const mrs_modules_msgs::OctomapPlannerDiagnostics &msg)
  {
    current_flag = msg.idle;

    // Check if this is the first message or if the flag has changed
    if (first_message || current_flag != previous_flag)
    {
        // Print the new flag value
        // ROS_INFO_STREAM("Flag value changed: " << (current_flag ? "True" : "False"));
        // Update the previous flag value
        previous_flag = current_flag;
        
        // After the first message, set the first_message flag to false
        first_message = false;
    }
  }

  void EGOReplanFSM::controllerDiagnosticsCallback(const mrs_msgs::ControlManagerDiagnostics &msg)
  {
    flying_normally_flag = msg.flying_normally;
  }

  void EGOReplanFSM::octoplannerPathCallback(const nav_msgs::Path &msg)
  {
    // if (receive_path_ == false)
    // {
    if (leader_swarm == false)
    {
      ROS_INFO("[SWARM_FSM]: Get a path from octomap planner.");
      get_path_ = msg;
      have_path_ = true;
    }
    else
    {
      have_path_ = false;
    }
    //   receive_path_ = true;
    // }
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "WAIT_FORMATION", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::formationWaypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (msg->pose.position.z < -0.1)
      return;

    ROS_INFO("[SWARM_FSM]: Triggered!");
    ROS_INFO("[SWARM_FSM]: Frame ID: %s", msg->header.frame_id.c_str());
    init_pt_ = odom_pos_;

    swarm_central_pos_(0) = msg->pose.position.x;
    swarm_central_pos_(1) = msg->pose.position.y;
    swarm_central_pos_(2) = uav_height;

    Eigen::Vector3d relative_pos;
    relative_pos << swarm_relative_pts_[id][0],
                    swarm_relative_pts_[id][1],
                    swarm_relative_pts_[id][2];
    end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos;

    ROS_INFO("[SWARM_FSM]: Final position [%d]: x: %f, y: %f, z: %f", id, float(end_pt_.x()), float(end_pt_.y()), float(end_pt_.z()));

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    have_target_ = true;

  }

  void EGOReplanFSM::formationCallback(const nav_msgs::Path::ConstPtr& msg)
  {
    std::vector<geometry_msgs::PoseStamped> poses = extractPosesFromPath(*msg);
    
    int j, path;
    if (path_number < 0)
    {
      j = 0;
      path = poses.size();
    }
    else
    {
      j = path_number;
      path = path_number + 1;
    }

    ROS_INFO("[SWARM_FSM]: Triggered!");

    for (int i = j; i < path; ++i)
    {
      const auto& pose = poses[i];

      if (pose.pose.position.z < -0.1)
        return;

      // ROS_INFO("[SWARM_FSM]: Frame ID: %s", msg->header.frame_id.c_str());
      ROS_INFO("[SWARM_FSM]: Path ID: %d", i);
      init_pt_ = odom_pos_;

      swarm_central_pos_(0) = pose.pose.position.x;
      swarm_central_pos_(1) = pose.pose.position.y;
      swarm_central_pos_(2) = uav_height;

      Eigen::Vector3d relative_pos;
      relative_pos << swarm_relative_pts_[id][0],
                      swarm_relative_pts_[id][1],
                      swarm_relative_pts_[id][2];
      end_pt_ = swarm_central_pos_ + swarm_scale_ * relative_pos;

      ROS_INFO("[SWARM_FSM]: Final position [%d]: x: %f, y: %f, z: %f", id, float(end_pt_.x()), float(end_pt_.y()), float(end_pt_.z()));

      visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

      // update
      end_pt_vector_.push_back(end_pt_);

      have_target_ = true;
    }

    end_pt_vector_size = end_pt_vector_.size();
    ROS_INFO("[SWARM_FSM]: Number of Poses: %d", end_pt_vector_size);

  }

  std::vector<geometry_msgs::PoseStamped> EGOReplanFSM::extractPosesFromPath(const nav_msgs::Path& path)
  {
    std::vector<geometry_msgs::PoseStamped> stampedPoses;

    // Iterate through the poses in the path
    for (const auto& poseStamped : path.poses) {
        stampedPoses.push_back(poseStamped);
    }

    return stampedPoses;
  }

  double EGOReplanFSM::calculateXYNorm(const Eigen::Vector3d& vector) 
  {
    return std::sqrt(vector.x() * vector.x() + vector.y() * vector.y());
  } 


} // namespace ego_planner
