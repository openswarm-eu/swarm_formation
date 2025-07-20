#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/Assignment.h>

#include <mrs_modules_msgs/OctomapPlannerDiagnostics.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <std_srvs/Trigger.h>
#include <actionlib/server/simple_action_server.h>
#include <ego_planner/ProcessPoseAction.h> 
#include <ego_planner/ProcessPoseGoal.h>
#include <ego_planner/ProcessPoseFeedback.h>
#include <ego_planner/ProcessPoseResult.h>

#include <fstream>
#include <iostream>
using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      WAIT_FORMATION,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      EMERGENCY_STOP,
      SEQUENTIAL_START
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET ,
      SWARM_MANUAL_TARGET 
    };
    
    /* planning utils */
    EGOPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoint_num_;
    int goal_num_;
    double goalpoints_[50][3];
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;
    bool enable_fail_safe_;
    int last_end_id_;
    double replan_trajectory_time_;

     // global goal setting for swarm
    Eigen::Vector3d swarm_central_pos_;
    double swarm_relative_pts_[50][3];
    double swarm_scale_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_swarm_relative_pts_, have_new_target_, have_recv_pre_agent_, have_local_traj_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    string odom_frame_id_;
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
    Eigen::Vector3d end_pt_, end_pt_tf_, end_vel_;                            // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                      // local target state
    int current_wp_;

    bool flag_escape_emergency_;
    bool flag_relan_astar_;

    GlobalTrajData frontend_traj_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, relative_pos_sub_, swarm_trajs_sub_, broadcast_bspline_sub_, trigger_sub_, assignment_sub_;
    ros::Publisher replan_pub_, new_pub_, poly_traj_pub_, data_disp_pub_, swarm_trajs_pub_, broadcast_bspline_pub_;
    ros::Publisher broadcast_ploytraj_pub_;
    ros::Publisher reached_pub_, start_pub_;
    ros::Subscriber central_goal;
    ros::Subscriber broadcast_ploytraj_sub_;
    // result file and file name
    string result_fn_;
    fstream result_file_;
    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj, bool use_formation); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init, bool use_formation);
    
    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    // void getLocalTarget();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void RecvBroadcastPolyTrajCallback(const traj_utils::PolyTrajConstPtr &msg);
    void polyTraj2ROSMsg(traj_utils::PolyTraj &msg);
    void formationWaypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    bool frontEndPathSearching();
    bool checkCollision();

    // MRS System
    ros::ServiceClient client;
    ros::ServiceClient client_octomap;
    ros::ServiceClient client_octomap_stop;
    ros::ServiceClient client_octomap_ref;
    ros::ServiceClient client_transform;
    ros::ServiceClient client_leader_transform;
    ros::Subscriber octoplanner_diagnostic_sub, controller_diagnostic_sub;
    void octoplannerDiagnosticsCallback(const mrs_modules_msgs::OctomapPlannerDiagnostics &msg);
    void controllerDiagnosticsCallback(const mrs_msgs::ControlManagerDiagnostics &msg);
    mrs_modules_msgs::OctomapPlannerDiagnostics diagnostics_;
    // Variable to store the previous state of the boolean flag
    bool previous_flag = false;
    // A flag to check if this is the first message received
    bool first_message = true;
    //
    bool current_flag, flying_normally_flag;
    ros::Subscriber octoplanner_path;
    void octoplannerPathCallback(const nav_msgs::Path &msg);
    bool send_service_, send_service_exe_, receive_path_, have_path_;
    Eigen::Vector3d swarm_central_path_;
    Eigen::Vector3d end_path_;
    nav_msgs::Path get_path_;

    string leader;
    int uav_number;
    // ros::Subscriber odom_sub2_, odom_sub3_;
    // void odometryCallback2(const nav_msgs::OdometryConstPtr &msg);
    // void odometryCallback3(const nav_msgs::OdometryConstPtr &msg);

    Eigen::Vector3d odom_pos2_, odom_vel2_, odom_acc2_; // odometry state
    Eigen::Vector3d odom_pos3_, odom_vel3_, odom_acc3_; // odometry state

    std::string name_state;
    bool dist_target, distance_type, leader_swarm;
    int id;

    // Add to deal with generic robots
    bool name_list;
    int number_of_robots;
    std::vector<std::string> robot_names;
    std::string name_robot, name_computer;
    double uav_height;
    double distance_target, distance_to_target;
    bool wait_takeoff, wait_odom;
    int path_number;

    // Add to deal with path from a client
    ros::Subscriber central_goal_topic;
    void formationCallback(const nav_msgs::Path::ConstPtr& msg);
    std::vector<geometry_msgs::PoseStamped> extractPosesFromPath(const nav_msgs::Path& path);
    std::vector<Eigen::Vector3d> end_pt_vector_;
    int end_pt_vector_size, end_pt_vector_id;
    double calculateXYNorm(const Eigen::Vector3d& vector);

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM();


    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif