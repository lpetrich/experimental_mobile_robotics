#include <game_navigation_layers/hiding_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

PLUGINLIB_EXPORT_CLASS(game_navigation_layers::HidingLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

double h_gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew){
    double dx = x - x0, dy = y - y0;
    double h = sqrt(dx * dx + dy * dy);
    double angle = atan2(dy, dx);
    double mx = cos(angle - skew) * h;
    double my = sin(angle - skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * varx), f2 = pow(my, 2.0)/(2.0 * vary);
    return A * exp(-(f1 + f2));
}

double h_get_radius(double cutoff, double A, double var){
    return sqrt(-2 * var * log(cutoff/A) );
}

namespace game_navigation_layers
{
    void HidingLayer::onInitialize()
    {
        HidingGameLayer::onInitialize();
        ros::NodeHandle nh("~/" + name_), g_nh;
        server_ = new dynamic_reconfigure::Server<HidingLayerConfig>(nh);
        f_ = boost::bind(&HidingLayer::configure, this, _1, _2);
        server_->setCallback(f_);
    }
    
    void HidingLayer::updateBoundsFromSeeker(double* min_x, double* min_y, double* max_x, double* max_y)
    {
        try {
            pos_b = seeker_position;
            vel_b = seeker_velocity;
             
            double mag = sqrt(pow(vel_b.x,2) + pow(vel_b.y, 2));
            double factor = 1.0 + mag * factor_;
            double point = h_get_radius(cutoff_, amplitude_, covar_ * factor );
              
            *min_x = std::min(*min_x, pos_b.x - point);
            *min_y = std::min(*min_y, pos_b.y - point);
            *max_x = std::max(*max_x, pos_b.x + point);
            *max_y = std::max(*max_y, pos_b.y + point);
        } catch(...) { return; }
    }
    
    void HidingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        boost::recursive_mutex::scoped_lock lock(lock_);

        if(!enabled_) { return; }
        if(!initialized_seeker_) { return; }
        if(cutoff_ >= amplitude_) { return; }
        
        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution();
        pos_c = seeker_position;
        vel_c = seeker_velocity;
        double angle = atan2(vel_c.y, vel_c.x);
        double mag = sqrt(pow(vel_c.x,2) + pow(vel_c.y, 2));
        double factor = 1.0 + mag * factor_;
        double base = h_get_radius(cutoff_, amplitude_, covar_);
        double point = h_get_radius(cutoff_, amplitude_, covar_ * factor ); 
        unsigned int width = std::max(1, int( (base + point) / res )), height = std::max(1, int( (base + point) / res ));
        double cx = pos_c.x, cy = pos_c.y;
        double ox, oy;

        if(sin(angle)>0)
            oy = cy - base;
        else
            oy = cy + (point-base) * sin(angle) - base;
        if(cos(angle)>=0)
            ox = cx - base;
        else
            ox = cx + (point-base) * cos(angle) - base;
        int dx, dy;
        costmap->worldToMapNoBounds(ox, oy, dx, dy);

        int start_x = 0, start_y = 0, end_x = width, end_y = height;
        if(dx < 0)
            start_x = -dx;
        else if(dx + width > costmap->getSizeInCellsX())
            end_x = std::max(0, (int)costmap->getSizeInCellsX() - dx);
        if((int)(start_x + dx) < min_i)
            start_x = min_i - dx;
        if((int)(end_x+dx) > max_i)
            end_x = max_i - dx;
        if(dy < 0)
            start_y = -dy;
        else if(dy + height > costmap->getSizeInCellsY())
            end_y = std::max(0, (int) costmap->getSizeInCellsY() - dy);
        if((int)(start_y + dy) < min_j)
            start_y = min_j - dy;
        if((int)(end_y + dy) > max_j)
            end_y = max_j - dy;

        double bx = ox + res / 2, by = oy + res / 2;
        for(int i = start_x; i < end_x; i++) {
            for(int j = start_y; j < end_y; j++) {
                unsigned char old_cost = costmap->getCost(i + dx, j + dy);
                if(old_cost == costmap_2d::NO_INFORMATION) { continue; }
                double x = bx + i * res, y = by + j * res;
                double ma = atan2(y - cy, x - cx);
                double diff = angles::shortest_angular_distance(angle, ma);
                double a;
                if(fabs(diff) < M_PI/2)
                    a = h_gaussian(x, y, cx, cy, amplitude_, covar_*factor, covar_, angle);
                else
                    a = h_gaussian(x, y, cx, cy, amplitude_, covar_, covar_, 0);
                if(a < cutoff_) { continue; }
                unsigned char cvalue = (unsigned char) a;
                costmap->setCost(i + dx, j + dy, std::max(cvalue, old_cost));
            }     
        }
    }

    void HidingLayer::configure(HidingLayerConfig &config, uint32_t level) {
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        covar_ = config.covariance;
        factor_ = config.factor;
        people_keep_time_ = ros::Duration(config.keep_time);
        enabled_ = config.enabled;
    }
};
