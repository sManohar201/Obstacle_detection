#include "../include/obstacle_detection/scan_merger.h"

using namespace obstacle_detection;
using namespace std;

ScanMerger::ScanMerger(ros::NodeHandle &nh, ros::NodeHandle &nh_local) : nh_(nh), nh_local_(nh_local) {
    p_active_ = false;

    front_scan_received_ = false;
    rear_scan_received_ = false;

    front_scan_error_ = false;
    rear_scan_error_ = false;

    params_srv_ = nh_local_.advertiseService("params", &ScanMerger::updateParams, this);

    initialize();
}

ScanMerger::~ScanMerger() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("publish_scan");
  nh_local_.deleteParam("publish_pcl");

  nh_local_.deleteParam("ranges_num");

  nh_local_.deleteParam("min_scanner_range");
  nh_local_.deleteParam("max_scanner_range");

  nh_local_.deleteParam("min_x_range");
  nh_local_.deleteParam("max_x_range");
  nh_local_.deleteParam("min_y_range");
  nh_local_.deleteParam("max_y_range");

  nh_local_.deleteParam("fixed_frame_id");
  nh_local_.deleteParam("target_frame_id");
}

bool ScanMerger::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("publish_scan", p_publish_scan_, false);
  nh_local_.param<bool>("publish_pcl", p_publish_pcl_, true);

  nh_local_.param<int>("ranges_num", p_ranges_num_, 1000);

  nh_local_.param<double>("min_scanner_range", p_min_scanner_range_, 0.05);
  nh_local_.param<double>("max_scanner_range", p_max_scanner_range_, 10.0);

  nh_local_.param<double>("min_x_range", p_min_x_range_, -10.0);
  nh_local_.param<double>("max_x_range", p_max_x_range_,  10.0);
  nh_local_.param<double>("min_y_range", p_min_y_range_, -10.0);
  nh_local_.param<double>("max_y_range", p_max_y_range_,  10.0);

  nh_local_.param<string>("fixed_frame_id", p_fixed_frame_id_, "map");
  nh_local_.param<string>("target_frame_id", p_target_frame_id_, "robot");

  if (prev_active != p_active_) {
      if(p_active_) {
          front_scan_sub_ = nh_.subscribe("front_scan", 10, &ScanMerger::frontScanCallback, this);
          rear_scan_sub_ = nh_.subscribe("rear_scan", 10, &ScanMerger::rearScanCallback, this);

          scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
          pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pcl", 10);
      } else {
          front_scan_sub_.shutdown();
          rear_scan_sub_.shutdown();

          scan_pub_.shutdown();
          pcl_pub_.shutdown();
      }
  }
  return true;
}

void ScanMerger::frontScanCallback(const sensor_msgs::LaserScan::ConstPtr front_scan) {
   try
   {
     tf_lf_.waitForTransform(front_scan->header.frame_id, p_fixed_frame_id_, 
                      front_scan->header.stamp + ros::Duration().fromNSec(front_scan->ranges.size()*front_scan->time_increment), ros::Duration(0.05));
    projector_.transformLaserScanToPointCloud(p_fixed_frame_id_, *front_scan, front_pcl_, tf_lf_);

   }
   catch(const tf::TransformException &e)
   {
     front_scan_error_ = true;
     return;
   }

   front_scan_received_ = true;
   front_scan_error_ = false;

   if (rear_scan_received_ || rear_scan_error_) {
     publishMessages();
   } else {
     rear_scan_error_ = true;
   }
}

void ScanMerger::rearScanCallback(const sensor_msgs::LaserScan::ConstPtr rear_scan) {
  try
  {
    tf_lf_.waitForTransform(rear_scan->header.frame_id, p_fixed_frame_id_, 
                            rear_scan->header.stamp + ros::Duration().fromNSec(rear_scan->ranges.size()*rear_scan->time_increment), ros::Duration(0.05));
    projector_.transformLaserScanToPointCloud(p_fixed_frame_id_, *rear_scan, rear_pcl_, tf_lf_);
  }
  catch(const tf::TransformException &e)
  {
    rear_scan_error_ = true;
  }
  
  rear_scan_received_ = true;
  rear_scan_error_ = false;

  if(front_scan_received_ || front_scan_error_) {
    publishMessages();
  } else {
    rear_scan_error_ = true;
  }
}

