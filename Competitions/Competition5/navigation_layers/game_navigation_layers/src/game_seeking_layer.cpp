#include <game_navigation_layers/game_seeking_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include "geometry_msgs/PointStamped.h"

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace game_navigation_layers
{
	void SeekingGameLayer::onInitialize()
	{
		ros::NodeHandle nh("~/" + name_), g_nh;
		current_ = true;
		first_time_ = true;
		pose_sub_ = nh.subscribe("/hider_pose", 1, &SeekingGameLayer::poseCallback, this);
		vel_sub_ = nh.subscribe("/hider_velocity", 1, &SeekingGameLayer::velocityCallback, this);
	}
	
	void SeekingGameLayer::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
		boost::recursive_mutex::scoped_lock lock(lock_);
		hider_pose.pose = msg->pose;
		hider_pose.header = msg->header;
	}

	void SeekingGameLayer::velocityCallback(const geometry_msgs::Point::ConstPtr& msg) {
		boost::recursive_mutex::scoped_lock lock(lock_);
		hider_velocity.x = msg->x;
		hider_velocity.y = msg->y;
		hider_velocity.z = msg->z;
	}

	void SeekingGameLayer::updateBounds(double origin_x, double origin_y, double origin_z, double* min_x, double* min_y, double* max_x, double* max_y){
		boost::recursive_mutex::scoped_lock lock(lock_);
		std::string global_frame = layered_costmap_->getGlobalFrameID();
		geometry_msgs::PointStamped pt, global_pt;			
		try {
			// update hider position
			pt.point.x = hider_pose.pose.pose.position.x;
			pt.point.y = hider_pose.pose.pose.position.y;
			pt.point.z = hider_pose.pose.pose.position.z;
			pt.header.frame_id = hider_pose.header.frame_id;
			tf_.transformPoint(global_frame, pt, global_pt);
			hider_position.x = global_pt.point.x;
			hider_position.y = global_pt.point.y;
			hider_position.z = global_pt.point.z;
			// update hider velocity
          	pt.point.x += hider_velocity.x;
          	pt.point.y += hider_velocity.y;
          	pt.point.z += hider_velocity.z;
      		tf_.transformPoint(global_frame, pt, global_pt);
          	hider_velocity.x = global_pt.point.x - hider_position.x;
          	hider_velocity.y = global_pt.point.y - hider_position.y;
          	hider_velocity.z = global_pt.point.z - hider_position.z;
          	if (!initialized_hider_) {
          		initialized_hider_ = true;
          	}
		}
		catch(tf::LookupException& ex) {
			ROS_ERROR("No Transform available Error: %s\n", ex.what());
			return;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_ERROR("Connectivity Error: %s\n", ex.what());
			return;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_ERROR("Extrapolation Error: %s\n", ex.what());
			return;
		}
		updateBoundsFromHider(min_x, min_y, max_x, max_y);
		if(first_time_){
				last_min_x_ = *min_x;
				last_min_y_ = *min_y;    
				last_max_x_ = *max_x;
				last_max_y_ = *max_y;    
				first_time_ = false;
		}else{
				double a = *min_x, b = *min_y, c = *max_x, d = *max_y;
				*min_x = std::min(last_min_x_, *min_x);
				*min_y = std::min(last_min_y_, *min_y);
				*max_x = std::max(last_max_x_, *max_x);
				*max_y = std::max(last_max_y_, *max_y);
				last_min_x_ = a;
				last_min_y_ = b;
				last_max_x_ = c;
				last_max_y_ = d;
		}
	}
};
