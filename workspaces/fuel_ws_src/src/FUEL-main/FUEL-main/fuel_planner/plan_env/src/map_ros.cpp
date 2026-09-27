#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include <fstream>

namespace fast_planner {
MapROS::MapROS() {
}

MapROS::~MapROS() {
}

void MapROS::setMap(SDFMap* map) {
  this->map_ = map;
}

void MapROS::init() {
  node_.param("map_ros/fx", fx_, -1.0);
  node_.param("map_ros/fy", fy_, -1.0);
  node_.param("map_ros/cx", cx_, -1.0);
  node_.param("map_ros/cy", cy_, -1.0);
  node_.param("map_ros/depth_filter_maxdist", depth_filter_maxdist_, -1.0);
  node_.param("map_ros/depth_filter_mindist", depth_filter_mindist_, -1.0);
  node_.param("map_ros/depth_filter_margin", depth_filter_margin_, -1);
  node_.param("map_ros/k_depth_scaling_factor", k_depth_scaling_factor_, -1.0);
  node_.param("map_ros/skip_pixel", skip_pixel_, -1);

  node_.param("map_ros/esdf_slice_height", esdf_slice_height_, -0.1);
  node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
  node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
  node_.param("map_ros/show_occ_time", show_occ_time_, false);
  node_.param("map_ros/show_esdf_time", show_esdf_time_, false);
  node_.param("map_ros/show_all_map", show_all_map_, false);
  node_.param("map_ros/frame_id", frame_id_, string("world"));

  proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  point_cloud_.points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  // proj_points_.reserve(640 * 480 / map_->mp_->skip_pixel_ / map_->mp_->skip_pixel_);
  proj_points_cnt = 0;

  local_updated_ = false;
  esdf_need_update_ = false;
  fuse_time_ = 0.0;
  esdf_time_ = 0.0;
  max_fuse_time_ = 0.0;
  max_esdf_time_ = 0.0;
  fuse_num_ = 0;
  esdf_num_ = 0;
  depth_image_.reset(new cv::Mat);

  rand_noise_ = normal_distribution<double>(0, 0.1);
  random_device rd;
  eng_ = default_random_engine(rd());

  esdf_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::updateESDFCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.05), &MapROS::visCallback, this);

  map_all_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_all", 10);
  map_local_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local", 10);
  map_local_inflate_pub_ =
      node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_local_inflate", 10);
  unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
  esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
  update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);

  // ========== 新增：注册 bounding box 话题 ==========
  box_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/bounding_box", 10);
  unknown_pub_2 = node_.advertise<visualization_msgs::Marker>("/map_ros/unknown_map", 1);
  // ==================================================

  depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/map_ros/depth", 50));
  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "/map_ros/cloud", 50));
  pose_sub_.reset(
      new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/map_ros/pose", 25));

  sync_image_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyImagePose>(
      MapROS::SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
  sync_image_pose_->registerCallback(boost::bind(&MapROS::depthPoseCallback, this, _1, _2));
  sync_cloud_pose_.reset(new message_filters::Synchronizer<MapROS::SyncPolicyCloudPose>(
      MapROS::SyncPolicyCloudPose(100), *cloud_sub_, *pose_sub_));
  sync_cloud_pose_->registerCallback(boost::bind(&MapROS::cloudPoseCallback, this, _1, _2));

  map_start_time_ = ros::Time::now();
}

void MapROS::visCallback(const ros::TimerEvent& e) {
  publishMapLocal();

  // ========== 新增：每次定时刷新都画一遍边框 ==========
  publishBoundingBox();
  // 在里面加上这行调用
publishUnknownMap();
  // ==================================================

  if (show_all_map_) {
    // Limit the frequency of all map
    static double tpass = 0.0;
    cout << "show all map!!!"<< endl;
    tpass += (e.current_real - e.last_real).toSec();
    if (tpass > 0.1) {
      publishMapAll();
      tpass = 0.0;
    }
  }
  // publishUnknown();
  // publishESDF();

  // publishUpdateRange();
  // publishDepth();
}

void MapROS::updateESDFCallback(const ros::TimerEvent& /*event*/) {
  if (!esdf_need_update_) return;
  auto t1 = ros::Time::now();

  map_->updateESDF3d();
  esdf_need_update_ = false;

  auto t2 = ros::Time::now();
  esdf_time_ += (t2 - t1).toSec();
  max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
  esdf_num_++;
  if (show_esdf_time_)
    ROS_WARN("ESDF t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), esdf_time_ / esdf_num_,
             max_esdf_time_);
}

void MapROS::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  if (!map_->isInMap(camera_pos_))  // exceed mapped region
    return;

  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                 pose->pose.orientation.y, pose->pose.orientation.z);
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  cv_ptr->image.copyTo(*depth_image_);

  auto t1 = ros::Time::now();

  // generate point cloud, update map
  proessDepthImage();
  map_->inputPointCloud(point_cloud_, proj_points_cnt, camera_pos_);
  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }

  auto t2 = ros::Time::now();
  fuse_time_ += (t2 - t1).toSec();
  max_fuse_time_ = max(max_fuse_time_, (t2 - t1).toSec());
  fuse_num_ += 1;
  if (show_occ_time_)
    ROS_WARN("Fusion t: cur: %lf, avg: %lf, max: %lf", (t2 - t1).toSec(), fuse_time_ / fuse_num_,
             max_fuse_time_);
}

void MapROS::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
                               const geometry_msgs::PoseStampedConstPtr& pose) {
  camera_pos_(0) = pose->pose.position.x;
  camera_pos_(1) = pose->pose.position.y;
  camera_pos_(2) = pose->pose.position.z;
  camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                 pose->pose.orientation.y, pose->pose.orientation.z);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  int num = cloud.points.size();

  map_->inputPointCloud(cloud, num, camera_pos_);

  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}

void MapROS::proessDepthImage() {
  proj_points_cnt = 0;

  uint16_t* row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
      depth = (*row_ptr) * inv_factor;
      row_ptr = row_ptr + skip_pixel_;

      // // filter depth
      // if (depth > 0.01)
      //   depth += rand_noise_(eng_);

      // TODO: simplify the logic here
      if (*row_ptr == 0 || depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;

      // pt_cur(0) = (u - cx_) * depth / fx_;
      // pt_cur(1) = (v - cy_) * depth / fy_;
      // pt_cur(2) = depth;
      pt_cur(0) = depth;
      pt_cur(1) = -(u - cx_) * depth / fx_;
      pt_cur(2) = -(v - cy_) * depth / fy_;
      pt_world = camera_r * pt_cur + camera_pos_;
      auto& pt = point_cloud_.points[proj_points_cnt++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }

  publishDepth();
}

void MapROS::publishMapAll() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud1.push_back(pt);
        }
      }
  cloud1.width = cloud1.points.size();
  cloud1.height = 1;
  cloud1.is_dense = true;
  cloud1.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud1, cloud_msg);
  map_all_pub_.publish(cloud_msg);

  // Output time and known volumn
  double time_now = (ros::Time::now() - map_start_time_).toSec();
  double known_volumn = 0;

  for (int x = map_->mp_->box_min_(0) /* + 1 */; x < map_->mp_->box_max_(0); ++x)
    for (int y = map_->mp_->box_min_(1) /* + 1 */; y < map_->mp_->box_max_(1); ++y)
      for (int z = map_->mp_->box_min_(2) /* + 1 */; z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->clamp_min_log_ - 1e-3)
          known_volumn += 0.1 * 0.1 * 0.1;
      }

  ofstream file("/home/boboyu/workspaces/plan_ws/src/fast_planner/exploration_manager/resource/"
                "curve1.txt",
                ios::app);
  file << "time:" << time_now << ",vol:" << known_volumn << std::endl;
}

void MapROS::publishMapLocal() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloud2;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  // for (int z = min_cut(2); z <= max_cut(2); ++z)
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] > map_->mp_->min_occupancy_log_) {
          // Occupied cells
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
        // else if (map_->md_->occupancy_buffer_inflate_[map_->toAddress(x, y, z)] == 1)
        // {
        //   // Inflated occupied cells
        //   Eigen::Vector3d pos;
        //   map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
        //   if (pos(2) > visualization_truncate_height_)
        //     continue;
        //   if (pos(2) < visualization_truncate_low_)
        //     continue;

        //   pt.x = pos(0);
        //   pt.y = pos(1);
        //   pt.z = pos(2);
        //   cloud2.push_back(pt);
        // }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  cloud2.width = cloud2.points.size();
  cloud2.height = 1;
  cloud2.is_dense = true;
  cloud2.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_local_pub_.publish(cloud_msg);
  pcl::toROSMsg(cloud2, cloud_msg);
  map_local_inflate_pub_.publish(cloud_msg);
}

void MapROS::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_cut = map_->md_->local_bound_min_;
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_;
  map_->boundIndex(max_cut);
  map_->boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] < map_->mp_->clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  unknown_pub_.publish(cloud_msg);
}

void MapROS::publishDepth() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i = 0; i < proj_points_cnt; ++i) {
    cloud.push_back(point_cloud_.points[i]);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  depth_pub_.publish(cloud_msg);
}

void MapROS::publishUpdateRange() {
  Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
  visualization_msgs::Marker mk;
  map_->indexToPos(map_->md_->local_bound_min_, esdf_min_pos);
  map_->indexToPos(map_->md_->local_bound_max_, esdf_max_pos);

  cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
  cube_scale = esdf_max_pos - esdf_min_pos;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;
  mk.pose.position.x = cube_pos(0);
  mk.pose.position.y = cube_pos(1);
  mk.pose.position.z = cube_pos(2);
  mk.scale.x = cube_scale(0);
  mk.scale.y = cube_scale(1);
  mk.scale.z = cube_scale(2);
  mk.color.a = 0.3;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;

  update_range_pub_.publish(mk);
}

void MapROS::publishESDF() {
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = 0.0;
  const double max_dist = 3.0;

  Eigen::Vector3i min_cut = map_->md_->local_bound_min_ - Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  Eigen::Vector3i max_cut = map_->md_->local_bound_max_ + Eigen::Vector3i(map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_,
                                                                          map_->mp_->local_map_margin_);
  map_->boundIndex(min_cut);
  map_->boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y) {
      Eigen::Vector3d pos;
      map_->indexToPos(Eigen::Vector3i(x, y, 1), pos);
      pos(2) = esdf_slice_height_;
      dist = map_->getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);
      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = -0.2;
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);

  // ROS_INFO("pub esdf");
}

void MapROS::publishUnknownMap() {
  visualization_msgs::Marker mk;
  // 直接使用你系统里已经有的 frame_id_
  mk.header.frame_id = frame_id_; 
  mk.header.stamp = ros::Time::now();
  mk.ns = "unknown_voxels";
  mk.id = 0;
  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.w = 1.0;

  // 获取网格分辨率
  double res = map_->mp_->resolution_;
  mk.scale.x = res;
  mk.scale.y = res;
  mk.scale.z = res;

  // 迷雾效果：灰色，30% 透明度
  mk.color.r = 0.6;
  mk.color.g = 0.6;
  mk.color.b = 0.6;
  mk.color.a = 0.3; 

  // 🚀 核心修正：严格按照你说的，只遍历黄框（探索边界）内的体素！
  for (int x = map_->mp_->box_min_(0); x < map_->mp_->box_max_(0); ++x) {
    for (int y = map_->mp_->box_min_(1); y < map_->mp_->box_max_(1); ++y) {
      for (int z = map_->mp_->box_min_(2); z < map_->mp_->box_max_(2); ++z) {
        
        // 获取这个体素在 1D 数组里的真实内存地址
        int address = map_->toAddress(x, y, z);
        
        // 🚀 核心判断：在 Log-Odds 地图中，0 就是绝对的“未观测/Unknown”
        // 只要扫到过，无论是变成墙(正数)还是空气(负数)，都不会是 0
        // ✅ 正确逻辑：原版 FUEL 的判断方式，小于最小截断值才是 Unknown！
if (map_->md_->occupancy_buffer_[address] < map_->mp_->clamp_min_log_ - 1e-3) { 
          
          Eigen::Vector3d pos;
          map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
          
          // 加上你系统自带的高度过滤，防止在 RViz 里渲染出天花板和地底下的迷雾，节省算力
          if (pos(2) > visualization_truncate_height_) continue;
          if (pos(2) < visualization_truncate_low_) continue;

          geometry_msgs::Point pt;
          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          mk.points.push_back(pt);
        }
      }
    }
  }

  // 如果黄框内还有未知区域，就发布出去
  if (!mk.points.empty()) {
    unknown_pub_2.publish(mk);
  } else {
    // 如果全探索完了，发一个空的清空历史显示
    mk.action = visualization_msgs::Marker::DELETEALL;
    unknown_pub_2.publish(mk);
  }
}

// ========== 新增：绘制 Bounding Box 的实现 ==========
void MapROS::publishBoundingBox() {
  visualization_msgs::Marker mk;
  mk.header.frame_id = frame_id_;  // 通常是 "world"
  mk.header.stamp = ros::Time::now();
  mk.ns = "exploration_bounding_box";
  mk.id = 0;
  
  // 使用 LINE_LIST (线段列表)，每两个点连成一条线，不遮挡地图
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.action = visualization_msgs::Marker::ADD;

  // 设置线条粗细
  mk.scale.x = 0.05; 

  // 设置线条颜色（这里设置为醒目的黄色，且不透明）
  mk.color.r = 1.0;
  mk.color.g = 1.0;
  mk.color.b = 0.0;
  mk.color.a = 1.0;

  // 从底层读取你设定的 Box 物理边界坐标 (box_mind_ 和 box_maxd_ 存的是 double 类型的米)
  double x_min = map_->mp_->box_mind_[0];
  double y_min = map_->mp_->box_mind_[1];
  double z_min = map_->mp_->box_mind_[2];
  double x_max = map_->mp_->box_maxd_[0];
  double y_max = map_->mp_->box_maxd_[1];
  double z_max = map_->mp_->box_maxd_[2];

  // 定义长方体的 8 个顶点坐标
  geometry_msgs::Point p[8];
  p[0].x = x_min; p[0].y = y_min; p[0].z = z_min;
  p[1].x = x_max; p[1].y = y_min; p[1].z = z_min;
  p[2].x = x_max; p[2].y = y_max; p[2].z = z_min;
  p[3].x = x_min; p[3].y = y_max; p[3].z = z_min;
  p[4].x = x_min; p[4].y = y_min; p[4].z = z_max;
  p[5].x = x_max; p[5].y = y_min; p[5].z = z_max;
  p[6].x = x_max; p[6].y = y_max; p[6].z = z_max;
  p[7].x = x_min; p[7].y = y_max; p[7].z = z_max;

  // 将顶点两两配对，塞入线条列表（12条边 = 24个点）
  // 底部四边形
  mk.points.push_back(p[0]); mk.points.push_back(p[1]);
  mk.points.push_back(p[1]); mk.points.push_back(p[2]);
  mk.points.push_back(p[2]); mk.points.push_back(p[3]);
  mk.points.push_back(p[3]); mk.points.push_back(p[0]);
  // 顶部四边形
  mk.points.push_back(p[4]); mk.points.push_back(p[5]);
  mk.points.push_back(p[5]); mk.points.push_back(p[6]);
  mk.points.push_back(p[6]); mk.points.push_back(p[7]);
  mk.points.push_back(p[7]); mk.points.push_back(p[4]);
  // 四根垂直柱子
  mk.points.push_back(p[0]); mk.points.push_back(p[4]);
  mk.points.push_back(p[1]); mk.points.push_back(p[5]);
  mk.points.push_back(p[2]); mk.points.push_back(p[6]);
  mk.points.push_back(p[3]); mk.points.push_back(p[7]);

  // 发布出去
  box_pub_.publish(mk);
}
// ==========================================================
}

