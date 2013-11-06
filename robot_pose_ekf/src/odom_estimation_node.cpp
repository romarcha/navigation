/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#include <robot_pose_ekf/odom_estimation_node.h>

using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;

static const double EPS = 1e-5;

namespace estimation
{
  // constructor
  OdomEstimationNode::OdomEstimationNode()
    : odom_active_(false),
      imu_active_(false),
      gps_active_(false),
      odom_initializing_(false),
      imu_initializing_(false),
      gps_initializing_(false),
      odom_covariance_(6),
      imu_covariance_(3),
      gps_covariance_(6),
      odom_callback_counter_(0),
      imu_callback_counter_(0),
      gps_callback_counter_(0),
      ekf_sent_counter_(0)
  {
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    // paramters
    nh_private.param("output_frame", output_frame_, std::string("odom_combined"));
    nh_private.param("sensor_timeout", timeout_, 1.0);
    nh_private.param("odom_used", odom_used_, true);
    nh_private.param("imu_used",  imu_used_, true);
    nh_private.param("gps_used",   gps_used_, true);
    nh_private.param("debug",   debug_, false);
    nh_private.param("self_diagnose",  self_diagnose_, false);
    double freq;
    nh_private.param("freq", freq, 30.0);

    timer_ = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &OdomEstimationNode::spin, this);

    // advertise our estimation
    pose_pub_ = nh_private.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_frame_, 10);

    // initialize
    filter_stamp_ = Time::now();

    // subscribe to odom messages
    if (odom_used_){
      ROS_DEBUG("Odom sensor can be used");
      odom_sub_ = nh.subscribe("odom", 10, &OdomEstimationNode::odomCallback, this);
    }
    else ROS_DEBUG("Odom sensor will NOT be used");

    // subscribe to imu messages
    if (imu_used_){
      ROS_DEBUG("Imu sensor can be used");
      imu_sub_ = nh.subscribe("imu_data", 10,  &OdomEstimationNode::imuCallback, this);
    }
    else ROS_DEBUG("Imu sensor will NOT be used");

    // subscribe to gps messages
    if (gps_used_){
      ROS_DEBUG("GPS sensor can be used");
      gps_sub_ = nh.subscribe("gps", 10, &OdomEstimationNode::gpsCallback, this);
    }
    else ROS_DEBUG("GPS sensor will NOT be used");

    // publish state service
    state_srv_ = nh_private.advertiseService("get_status", &OdomEstimationNode::getStatus, this);

    if (debug_){
      // open files for debugging
      odom_file_.open("/tmp/odom_file.txt");
      imu_file_.open("/tmp/imu_file.txt");
      gps_file_.open("/tmp/gps_file.txt");
      corr_file_.open("/tmp/corr_file.txt");
      time_file_.open("/tmp/time_file.txt");
      extra_file_.open("/tmp/extra_file.txt");
    }
  };

  // destructor
  OdomEstimationNode::~OdomEstimationNode(){

    if (debug_){
      // close files for debugging
      odom_file_.close();
      imu_file_.close();
      gps_file_.close();
      corr_file_.close();
      time_file_.close();
      extra_file_.close();
    }
  };

  // callback function for odom data
  void OdomEstimationNode::odomCallback(const OdomConstPtr& odom)
  {
    odom_callback_counter_++;

    ROS_DEBUG("Odom callback at time %f ", ros::Time::now().toSec());
    assert(odom_used_);

    // receive data 
    odom_stamp_ = odom->header.stamp;
    odom_time_  = Time::now();
    Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        odom_covariance_(i+1, j+1) = odom->pose.covariance[6*i+j];

    my_filter_.addMeasurement(StampedTransform(odom_meas_.inverse(), odom_stamp_, "base_footprint", "wheelodom"), odom_covariance_);
    
    // activate odom
    if (!odom_active_) {
      if (!odom_initializing_){
	odom_initializing_ = true;
	odom_init_stamp_ = odom_stamp_;
	ROS_INFO("Initializing Odom sensor");      
      }
      if ( filter_stamp_ >= odom_init_stamp_){
	odom_active_ = true;
	odom_initializing_ = false;
	ROS_INFO("Odom sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate Odom, because Odom measurements are still %f sec in the future.", 
		    (odom_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double tmp, yaw;
      odom_meas_.getBasis().getEulerYPR(yaw, tmp, tmp);
      odom_file_ << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
    }
  };

  // callback function for imu data
  void OdomEstimationNode::imuCallback(const ImuConstPtr& imu)
  {
    imu_callback_counter_++;

    assert(imu_used_);

    // receive data 
    imu_stamp_ = imu->header.stamp;
    tf::Quaternion orientation;
    quaternionMsgToTF(imu->orientation, orientation);
    imu_meas_ = tf::Transform(orientation, tf::Vector3(0,0,0));
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        imu_covariance_(i+1, j+1) = imu->orientation_covariance[3*i+j];

    // Transforms imu data to base_footprint frame
    if (!robot_state_.waitForTransform("base_footprint", imu->header.frame_id, imu_stamp_, ros::Duration(0.5))){
      // warn when imu was already activated, not when imu is not active yet
      if (imu_active_)
        ROS_ERROR("Could not transform imu message from %s to base_footprint", imu->header.frame_id.c_str());
      else if (my_filter_.isInitialized())
        ROS_WARN("Could not transform imu message from %s to base_footprint. Imu will not be activated yet.", imu->header.frame_id.c_str());
      else 
        ROS_DEBUG("Could not transform imu message from %s to base_footprint. Imu will not be activated yet.", imu->header.frame_id.c_str());
      return;
    }
    StampedTransform base_imu_offset;
    robot_state_.lookupTransform("base_footprint", imu->header.frame_id, imu_stamp_, base_imu_offset);
    imu_meas_ = imu_meas_ * base_imu_offset;

    imu_time_  = Time::now();

    // manually set covariance untile imu sends covariance
    if (imu_covariance_(1,1) == 0.0){
      SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
      measNoiseImu_Cov(1,1) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(2,2) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(3,3) = pow(0.00017,2);  // = 0.01 degrees / sec
      imu_covariance_ = measNoiseImu_Cov;
    }

    my_filter_.addMeasurement(StampedTransform(imu_meas_.inverse(), imu_stamp_, "base_footprint", "imu"), imu_covariance_);
    
    // activate imu
    if (!imu_active_) {
      if (!imu_initializing_){
	imu_initializing_ = true;
	imu_init_stamp_ = imu_stamp_;
	ROS_INFO("Initializing Imu sensor");      
      }
      if ( filter_stamp_ >= imu_init_stamp_){
	imu_active_ = true;
	imu_initializing_ = false;
	ROS_INFO("Imu sensor activated");      
      }
      else ROS_DEBUG("Waiting to activate IMU, because IMU measurements are still %f sec in the future.", 
		    (imu_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double tmp, yaw;
      imu_meas_.getBasis().getEulerYPR(yaw, tmp, tmp); 
      imu_file_ << yaw << endl;
    }
  };

  // callback function for GPS data
  void OdomEstimationNode::gpsCallback(const GPSConstPtr& gps)
  {
    gps_callback_counter_++;

    assert(gps_used_);

    // get data
    gps_stamp_ = gps->header.stamp;
    gps_time_  = Time::now();
    poseMsgToTF(gps->pose.pose, gps_meas_);
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        gps_covariance_(i+1, j+1) = gps->pose.covariance[6*i+j];
    my_filter_.addMeasurement(StampedTransform(gps_meas_.inverse(), gps_stamp_, "base_footprint", "gps"), gps_covariance_);
    
    // activate gps
    if (!gps_active_) {
      if (!gps_initializing_){
    gps_initializing_ = true;
    gps_init_stamp_ = gps_stamp_;
    ROS_INFO("Initializing Gps sensor");
      }
      if (filter_stamp_ >= gps_init_stamp_){
    gps_active_ = true;
    gps_initializing_ = false;
    ROS_INFO("Gps sensor activated");
      }
      else ROS_DEBUG("Waiting to activate GPS, because GPS measurements are still %f sec in the future.",
            (gps_init_stamp_ - filter_stamp_).toSec());
    }
    
    if (debug_){
      // write to file
      double Rx, Ry, Rz;
      gps_meas_.getBasis().getEulerYPR(Rz, Ry, Rx);
      gps_file_ << gps_meas_.getOrigin().x() << " " << gps_meas_.getOrigin().y() << " " << gps_meas_.getOrigin().z() << " "
               << Rx << " " << Ry << " " << Rz << endl;
    }
  };

  // filter loop
  void OdomEstimationNode::spin(const ros::TimerEvent& e)
  {
    ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());

    // check for timing problems
    if ( (odom_initializing_ || odom_active_) && (imu_initializing_ || imu_active_) ){
      double diff = fabs( Duration(odom_stamp_ - imu_stamp_).toSec() );
      if (diff > 1.0) ROS_ERROR("Timestamps of odometry and imu are %f seconds apart.", diff);
    }
    
    // initial value for filter stamp; keep this stamp when no sensors are active
    filter_stamp_ = Time::now();
    
    // check which sensors are still active
    if ((odom_active_ || odom_initializing_) && 
        (Time::now() - odom_time_).toSec() > timeout_){
      odom_active_ = false; odom_initializing_ = false;
      ROS_INFO("Odom sensor not active any more");
    }
    if ((imu_active_ || imu_initializing_) && 
        (Time::now() - imu_time_).toSec() > timeout_){
      imu_active_ = false;  imu_initializing_ = false;
      ROS_INFO("Imu sensor not active any more");
    }
    if ((gps_active_ || gps_initializing_) &&
        (Time::now() - gps_time_).toSec() > timeout_){
      gps_active_ = false;  gps_initializing_ = false;
      ROS_INFO("GPS sensor not active any more");
    }
    
    // only update filter when one of the sensors is active
    if (odom_active_ || imu_active_ || gps_active_){
      
      // update filter at time where all sensor measurements are available
      if (odom_active_)  filter_stamp_ = min(filter_stamp_, odom_stamp_);
      if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
      if (gps_active_)    filter_stamp_ = min(filter_stamp_, gps_stamp_);
      
      // update filter
      if ( my_filter_.isInitialized() )  {
        bool diagnostics = true;
        if (my_filter_.update(odom_active_, imu_active_, gps_active_,  filter_stamp_, diagnostics)){
          
          // output most recent estimate and relative covariance
          my_filter_.getEstimate(output_);
          pose_pub_.publish(output_);
          ekf_sent_counter_++;
          
          // broadcast most recent estimate to TransformArray
          StampedTransform tmp;
          my_filter_.getEstimate(ros::Time(), tmp);
          if(!gps_active_)
            tmp.getOrigin().setZ(0.0);
          odom_broadcaster_.sendTransform(StampedTransform(tmp, tmp.stamp_, output_frame_, "base_footprint"));
          
          if (debug_){
            // write to file
            ColumnVector estimate; 
            my_filter_.getEstimate(estimate);
            for (unsigned int i=1; i<=6; i++)
              corr_file_ << estimate(i) << " ";
            corr_file_ << endl;
          }
        }
        if (self_diagnose_ && !diagnostics)
          ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
      }
      // initialize filer with odometry frame
      if ( odom_active_ && !my_filter_.isInitialized()){
        my_filter_.initialize(odom_meas_, odom_stamp_);
        ROS_INFO("Kalman filter initialized with odom measurement");
      }
      else if ( gps_active_ && !my_filter_.isInitialized()){
        my_filter_.initialize(gps_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps measurement");
      }
    }
  };

bool OdomEstimationNode::getStatus(robot_pose_ekf::GetStatus::Request& req, robot_pose_ekf::GetStatus::Response& resp)
{
  stringstream ss;
  ss << "Input:" << endl;
  ss << " * Odometry sensor" << endl;
  ss << "   - is "; if (!odom_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << odom_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << odom_sub_.getTopic() << endl;
  ss << " * IMU sensor" << endl;
  ss << "   - is "; if (!imu_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << imu_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << imu_sub_.getTopic() << endl;
  ss << " * Visual Odometry sensor" << endl;
  ss << "   - is "; if (!gps_active_) ss << "NOT "; ss << "active" << endl;
  ss << "   - received " << gps_callback_counter_ << " messages" << endl;
  ss << "   - listens to topic " << gps_sub_.getTopic() << endl;
  ss << "Output:" << endl;
  ss << " * Robot pose ekf filter" << endl;
  ss << "   - is "; if (!my_filter_.isInitialized()) ss << "NOT "; ss << "active" << endl;
  ss << "   - sent " << ekf_sent_counter_ << " messages" << endl;
  ss << "   - pulishes on topics " << pose_pub_.getTopic() << " and /tf" << endl;
  resp.status = ss.str();
  return true;
}

}; // namespace



// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "robot_pose_ekf");

  // create filter class
  OdomEstimationNode my_filter_node;

  ros::spin();
  
  return 0;
}
