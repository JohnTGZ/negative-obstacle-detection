#ifndef CLIFF_DETECTOR_H
#define CLIFF_DETECTOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>

#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

#include "laser_geometry/laser_geometry.h"


class CliffDetector{
    public:
        CliffDetector();
        ~CliffDetector();

        /**
         * @brief Initialize publishers and subscribers
         */
        void initPubSubSrv();

        /**
         * @brief Read params from ROSParam server
         * and initialize other internal parameters
         */
        void initParams();
        
        /**
         * @brief Generates lookup table to compare 
         * scan messages against their unobstructed values
         * It is assumed that the scan is symmetrical about the 
         * x axis of of the lidar
         * @param scan_size size of laser scan message
         * @return lookup table of thresholded laser scan values
         */
        std::vector<float> generateCliffLookup(int scan_size);
        
        /**
         * @brief Overrides the existing lidar properties 
         * @param _lidar_pitch_deg pitch of lidar (about y axis)
         * @param _lidar_height height of lidar from base footprint
         * @param _lidar_resolution resolution of lidar scan
         */
        void setLidarParams(double _lidar_pitch_deg, 
                                double _lidar_height, 
                                double _lidar_resolution);

        /**
         * @brief Overrides the failsafe params
         * @param _failsafe 
         * @param _failsafe_threshold_dist
         * @param _failsafe_threshold_percentage 
         * @param _failsafe_timeout 
         */
        void setFailsafeParams( bool    _failsafe, 
                                double _failsafe_threshold_dist, 
                                double _failsafe_threshold_percentage,
                                double _failsafe_timeout);

        /**
         * @brief Overrides the cliff params
         * @param _cliff_thresh_constant 
         * @param _min_cliff_pts_in_seg 
         */
        void setCliffParams(    double _cliff_thresh_constant, 
                                int _min_cliff_pts_in_seg);

        /**
         * @brief Detects cliff points in scan message
         * @param scan_in filtered scan message from 
         * cliff sensor
         */
        void detectCliff(sensor_msgs::LaserScan& scan, std::vector<float>& _lookup_thresh_range);

        sensor_msgs::LaserScan scan_out;
        sensor_msgs::LaserScan scan_thresh;

        //flags
        bool failsafe_activated_;

    private:
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        /**
         * @brief Callback for obtaining cliff obstacles
         * scan messages
         * @param scan_in filtered scan message from 
         * cliff sensor
         */
        void cliffScanCB(const sensor_msgs::LaserScan::ConstPtr& scan_in);
        
        /**
         * @brief Failsafe feature to stop the robot 
         * when lidar is obstructed
         */
        void cmd_velStop();

        //Vector of distances traveled by unobstructued 
        //rays from lidar to ground plane 
        std::vector<float> lookup_thresh_range_;

        ///////////////////////////
        //ROSParams
        ///////////////////////////
        std::string scan_input_;
        std::string scan_output_;

        bool failsafe_;
        double failsafe_threshold_dist_;
        double failsafe_threshold_percentage_;
        int failsafe_max_count_;
        double  failsafe_timeout_;

        ros::Time time_elapsed;

        double cliff_thresh_constant_;

        int min_cliff_pts_in_seg_;

        double lidar_height_;
        double lidar_pitch_deg_;
        double lidar_resolution_;

        std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr;

        //////////////////////////////
        //Internal state machine flags
        //////////////////////////////
        bool first_scan_cb_;

        ///////////////////////////
        //Subscribers & Publishers
        ///////////////////////////

        ros::Subscriber scan_sub_;
        ros::Publisher scan_cliff_pub_;
        ros::Publisher scan_thresh_pub;
        ros::Publisher cmd_vel_pub_;

        ///////////////////////////
        //laser geometry & transforms
        ///////////////////////////
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;

};

#endif //CLIFF_DETECTOR_H
