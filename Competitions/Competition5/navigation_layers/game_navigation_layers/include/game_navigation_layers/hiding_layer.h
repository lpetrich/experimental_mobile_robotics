#ifndef HIDING_LAYER_H_
#define HIDING_LAYER_H_
#include <ros/ros.h>
#include <game_navigation_layers/game_hiding_layer.h>
#include <dynamic_reconfigure/server.h>
#include <game_navigation_layers/HidingLayerConfig.h>
#include "geometry_msgs/Point.h"

double h_gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double h_get_radius(double cutoff, double A, double var);

namespace game_navigation_layers
{
	class HidingLayer : public HidingGameLayer
	{
		public:
			HidingLayer() { layered_costmap_ = NULL; }

			virtual void onInitialize();
			virtual void updateBoundsFromSeeker(double* min_x, double* min_y, double* max_x, double* max_y);
			virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

		protected:
			void configure(HidingLayerConfig &config, uint32_t level);
			double cutoff_, amplitude_, covar_, factor_;
			ros::Duration people_keep_time_;
			geometry_msgs::Point pos_c;
			geometry_msgs::Point vel_c;			
			geometry_msgs::Point pos_b;
			geometry_msgs::Point vel_b;
			dynamic_reconfigure::Server<HidingLayerConfig>* server_;
			dynamic_reconfigure::Server<HidingLayerConfig>::CallbackType f_;
	};
};

#endif

