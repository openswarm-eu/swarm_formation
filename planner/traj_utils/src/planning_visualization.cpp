#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace ego_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;

    goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
    global_list_pub = nh.advertise<visualization_msgs::Marker>("global_list", 2);
    init_list_pub = nh.advertise<visualization_msgs::Marker>("init_list", 2);
    optimal_list_pub = nh.advertise<visualization_msgs::Marker>("optimal_list", 2);
    failed_list_pub = nh.advertise<visualization_msgs::Marker>("failed_list", 2);
    a_star_list_pub = nh.advertise<visualization_msgs::Marker>("a_star_list", 20);
    init_list_debug_pub = nh.advertise<visualization_msgs::Marker>("init_debug_list",2);
    
    intermediate_pt0_pub = nh.advertise<visualization_msgs::Marker>("pt0_dur_opt", 10);
    intermediate_grad0_pub = nh.advertise<visualization_msgs::MarkerArray>("grad0_dur_opt", 10);
    intermediate_pt1_pub = nh.advertise<visualization_msgs::Marker>("pt1_dur_opt", 10);
    intermediate_grad1_pub = nh.advertise<visualization_msgs::MarkerArray>("grad1_dur_opt", 10);
    intermediate_grad_smoo_pub = nh.advertise<visualization_msgs::MarkerArray>("smoo_grad_dur_opt", 10);
    intermediate_grad_dist_pub = nh.advertise<visualization_msgs::MarkerArray>("dist_grad_dur_opt", 10);
    intermediate_grad_feas_pub = nh.advertise<visualization_msgs::MarkerArray>("feas_grad_dur_opt", 10);
    intermediate_grad_swarm_pub = nh.advertise<visualization_msgs::MarkerArray>("swarm_grad_dur_opt", 10);

    swarm_formation_visual_pub  = nh.advertise<visualization_msgs::MarkerArray>("swarm_graph_visual", 10);
    
    t_init = ros::Time::now();
    
    nh.param("manager/drone_id", drone_id_, -1);
    nh.param("optimization/formation_type", formation_type_, -1);
    initSwarmGraphVisual();
    swarm_odom.resize(formation_size_);
    for (int i=0; i<formation_size_; i++)
      swarm_odom[i] = Eigen::Vector3d::Zero();
    
    drone_0_odom_sub_ = nh.subscribe("/drone_0_visual_slam/odom", 1, &PlanningVisualization::drone_0_odomeCallback, this);
    drone_1_odom_sub_ = nh.subscribe("/drone_1_visual_slam/odom", 1, &PlanningVisualization::drone_1_odomeCallback, this);
    drone_2_odom_sub_ = nh.subscribe("/drone_2_visual_slam/odom", 1, &PlanningVisualization::drone_2_odomeCallback, this);
    drone_3_odom_sub_ = nh.subscribe("/drone_3_visual_slam/odom", 1, &PlanningVisualization::drone_3_odomeCallback, this);
    drone_4_odom_sub_ = nh.subscribe("/drone_4_visual_slam/odom", 1, &PlanningVisualization::drone_4_odomeCallback, this);
    drone_5_odom_sub_ = nh.subscribe("/drone_5_visual_slam/odom", 1, &PlanningVisualization::drone_5_odomeCallback, this);
    drone_6_odom_sub_ = nh.subscribe("/drone_6_visual_slam/odom", 1, &PlanningVisualization::drone_6_odomeCallback, this);
    drone_7_odom_sub_ = nh.subscribe("/drone_7_visual_slam/odom", 1, &PlanningVisualization::drone_7_odomeCallback, this);
    drone_8_odom_sub_ = nh.subscribe("/drone_8_visual_slam/odom", 1, &PlanningVisualization::drone_8_odomeCallback, this);
    drone_9_odom_sub_ = nh.subscribe("/drone_9_visual_slam/odom", 1, &PlanningVisualization::drone_9_odomeCallback, this);
    drone_10_odom_sub_ = nh.subscribe("/drone_10_visual_slam/odom", 1, &PlanningVisualization::drone_10_odomeCallback, this);
    drone_11_odom_sub_ = nh.subscribe("/drone_11_visual_slam/odom", 1, &PlanningVisualization::drone_11_odomeCallback, this);
    
    
    if (drone_id_ == 0){
      swarm_graph_visual_timer_ = nh.createTimer(ros::Duration(0.01), &PlanningVisualization::swarmGraphVisulCallback, this);
    }
  }
  
  void PlanningVisualization::swarmGraphVisulCallback(const ros::TimerEvent &e){
    if (line_size_==0)
      return;
    
    visualization_msgs::MarkerArray lines;
    for (int i=0; i<line_size_; i++){
      visualization_msgs::Marker line_strip;
      line_strip.header.frame_id = "world";
      line_strip.header.stamp = ros::Time::now();
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_strip.action = visualization_msgs::Marker::ADD;
      line_strip.id = i;
      
      // line_strip.scale.x = 0.05;
      // line_strip.color.r = 0.0;
      // line_strip.color.g = 0.5;
      // line_strip.color.b = 0.5;
      // line_strip.color.a = 0.7;

      line_strip.scale.x = 0.2;
      line_strip.color.r = 0.9;
      line_strip.color.g = 0.3;
      line_strip.color.b = 0.3;
      line_strip.color.a = 0.8;

      geometry_msgs::Point p, q;
      p.x = swarm_odom[line_begin_[i]](0);
      p.y = swarm_odom[line_begin_[i]](1);
      p.z = swarm_odom[line_begin_[i]](2);

      q.x = swarm_odom[line_end_[i]](0);
      q.y = swarm_odom[line_end_[i]](1);
      q.z = swarm_odom[line_end_[i]](2);
      line_strip.points.push_back(p);        
      line_strip.points.push_back(q);

      lines.markers.push_back(line_strip);
    }
    swarm_formation_visual_pub.publish(lines);
  }

  void PlanningVisualization::benchmarkCallback(const ros::TimerEvent &e){
       
       t_record = ros::Time::now();
       double t_current = (t_record - t_init).toSec();
       odom_csv << t_current << ",";
       for( auto odom: swarm_odom){
         odom_csv << odom(0) << "," << odom(1) << ", ";
       }
       odom_csv << std::endl;
  }

  void PlanningVisualization::drone_0_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=0 )
      return;
    
    swarm_odom[0] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_1_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=1 )
      return;
    
    swarm_odom[1] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_2_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=2 )
      return;
    
    swarm_odom[2] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_3_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=3 )
      return;
    
    swarm_odom[3] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_4_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=4 )
      return;
    
    swarm_odom[4] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_5_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=5 )
      return;
    
    swarm_odom[5] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_6_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=5 )
      return;
    
    swarm_odom[6] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_7_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=7 )
      return;
    
    swarm_odom[7] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_8_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=8 )
      return;
    
    swarm_odom[8] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_9_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=9 )
      return;
    
    swarm_odom[9] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_10_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=10 )
      return;
    
    swarm_odom[10] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::drone_11_odomeCallback(const nav_msgs::OdometryConstPtr &msg){
    if (formation_size_ <=11 )
      return;
    
    swarm_odom[11] << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
  }

  void PlanningVisualization::initSwarmGraphVisual(){
    switch (formation_type_)
    {
    case FORMATION_TYPE::NONE_FORMATION:
    {
      formation_size_ = 0;
      line_size_      = 0;
      break;
    }

    case FORMATION_TYPE::REGULAR_HEXAGON:
    {
      formation_size_ = 7;
      line_size_      = 12;
      line_begin_.resize(line_size_);
      line_end_.resize(line_size_);
      line_begin_ = {0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6};
      line_end_   = {1, 2, 3, 4, 5, 6, 2, 3, 4, 5, 6, 1};
      
      break;
    }

    case FORMATION_TYPE::REGULAR_TRIANGLE:
    {
      formation_size_ = 3;
      line_size_      = 3;
      line_begin_.resize(line_size_);
      line_end_.resize(line_size_);
      line_begin_ = {0, 0, 1};
      line_end_   = {1, 2, 2};

      break;
    }

    case FORMATION_TYPE::REGULAR_LINE:
    {
      formation_size_ = 2;
      line_size_      = 1;
      line_begin_.resize(line_size_);
      line_end_.resize(line_size_);
      line_begin_ = {0};
      line_end_   = {1};

      break;
    }
    
    default:
      break;
    }
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      if (show_sphere) sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time::now();
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "world";
    sphere.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = goal_point(2);

    goal_point_pub.publish(sphere);
  }

  void PlanningVisualization::displayGlobalPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (global_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0.5, 0.5, 1);
    displayMarkerList(global_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    static int last_nums = 0;

    for ( int id=0; id<last_nums; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(init_list_pub, blank, scale, color, id, false);
      ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for ( int id=0; id<(int)init_trajs.size(); id++ )
    {
      Eigen::Vector4d color(0, 0, 1, 0.7);
      displayMarkerList(init_list_pub, init_trajs[id], scale, color, id, false);
      ros::Duration(0.001).sleep();
      last_nums++;
    }

  }

  void PlanningVisualization::displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (init_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_list_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayInitPathListDebug(vector<Eigen::Vector3d> init_pts, const double scale, int id)
  {

    if (init_list_debug_pub.getNumSubscribers() == 0)
    {
      return;
    }

    Eigen::Vector4d color(1, 1, 0, 1);
    displayMarkerList(init_list_debug_pub, init_pts, scale, color, id);
  }

  void PlanningVisualization::displayOptimalList(Eigen::MatrixXd optimal_pts, int id)
  {

    if (optimal_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < optimal_pts.cols(); i++)
    {
      Eigen::Vector3d pt = optimal_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(1.0, 0.0, 0, 0.8);
    displayMarkerList(optimal_list_pub, list, 0.1, color, id);
    // displayMarkerList(optimal_list_pub, list, 0.08, color, id);
  }

  void PlanningVisualization::displayFailedList(Eigen::MatrixXd failed_pts, int id)
  {

    if (failed_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> list;
    for (int i = 0; i < failed_pts.cols(); i++)
    {
      Eigen::Vector3d pt = failed_pts.col(i).transpose();
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.3, 0, 0, 1);
    displayMarkerList(failed_list_pub, list, 0.15, color, id);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub.getNumSubscribers() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths)
    {
      list.clear();
      for (auto pt : block)
      {
        list.push_back(pt);
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      displayMarkerList(a_star_list_pub, list, scale, color, id + i); // real ids used: [ id ~ id+a_star_paths.size() ]
      i++;
    }
  }

  void PlanningVisualization::displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::MarkerArray array;
    // clear
    pub.publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub.publish(array);
  }

  void PlanningVisualization::displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color)
  {
    std::vector<Eigen::Vector3d> pts_;
    pts_.reserve(pts.cols());
    for ( int i=0; i<pts.cols(); i++ )
    {
      pts_.emplace_back(pts.col(i));
    }

    if ( !type.compare("0") )
    {
      displayMarkerList(intermediate_pt0_pub, pts_, 0.1, color, id);
    }
    else if ( !type.compare("1") )
    {
      displayMarkerList(intermediate_pt1_pub, pts_, 0.1, color, id);
    }
  }

  void PlanningVisualization::displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color)
  {
    if ( pts.cols() != grad.cols() )
    {
      ROS_ERROR("pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector3d> arrow_;
    arrow_.reserve(pts.cols()*2);
    if ( !type.compare("swarm") )
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(grad.col(i));
      }
    }
    else
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(pts.col(i)+grad.col(i));
      }
    }
    

    if ( !type.compare("grad0") )
    {
      displayArrowList(intermediate_grad0_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("grad1") )
    {
      displayArrowList(intermediate_grad1_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("dist") )
    {
      displayArrowList(intermediate_grad_dist_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("smoo") )
    {
      displayArrowList(intermediate_grad_smoo_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("feas") )
    {
      displayArrowList(intermediate_grad_feas_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("swarm") )
    {
      displayArrowList(intermediate_grad_swarm_pub, arrow_, 0.02, color, id);
    }
    
  }

  // PlanningVisualization::
} // namespace ego_planner