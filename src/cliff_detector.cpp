#include "cliff_detector/cliff_detector.h"

#define PI 3.141592653589793
#define DEG2RAD 0.0174532

CliffDetector::CliffDetector(): nh(), pnh("~"){
    time_elapsed = ros::Time::now();
    initParams();
    initPubSubSrv();
}

CliffDetector::~CliffDetector(){
}

void CliffDetector::setLidarParams(double _lidar_pitch_deg, 
                                        double _lidar_height, 
                                        double _lidar_resolution){
    lidar_pitch_deg_ = _lidar_pitch_deg;
    lidar_height_ = _lidar_height;
    lidar_resolution_ = _lidar_resolution;
}

void CliffDetector::setFailsafeParams(  bool    _failsafe, 
                                        double _failsafe_threshold_dist, 
                                        double _failsafe_threshold_percentage,
                                        double _failsafe_timeout){
    failsafe_ = _failsafe;
    failsafe_threshold_dist_ = _failsafe_threshold_dist;
    failsafe_threshold_percentage_ = _failsafe_threshold_percentage;
    failsafe_timeout_ = _failsafe_timeout;
}

void CliffDetector::setCliffParams( double _cliff_thresh_constant, 
                                    int _min_cliff_pts_in_seg){
    cliff_thresh_constant_ = _cliff_thresh_constant;
    min_cliff_pts_in_seg_ = _min_cliff_pts_in_seg;
}


void CliffDetector::initParams(){
    pnh.param<std::string>("scan", scan_input_, "scan");
    pnh.param<std::string>("scan_out", scan_output_, "scan_out");

    //lidar params
    pnh.param<double>("lidar_height", lidar_height_, 0.925);
    pnh.param<double>("lidar_pitch", lidar_pitch_deg_, 30.0);
    pnh.param<double>("lidar_resolution", lidar_resolution_, 1.0);

    //failsafe params
    pnh.param<bool>("failsafe", failsafe_, true);
    pnh.param<double>("failsafe_threshold_dist", failsafe_threshold_dist_, 0.2);
    pnh.param<double>("failsafe_threshold_percentage", failsafe_threshold_percentage_, 0.4);
    pnh.param<double>("failsafe_timeout", failsafe_timeout_, 2.0);

    //cliff detection params
    pnh.param<double>("cliff_threshold_constant", cliff_thresh_constant_, 0.20);
    pnh.param<int>("min_cliff_pts_in_seg", min_cliff_pts_in_seg_, 5);

    if (lidar_pitch_deg_ < 0 || lidar_height_ < 0){
        ROS_ERROR("Please make sure lidar pitch (%f) and height (%f) is positive!", lidar_pitch_deg_, lidar_height_);
    }

    ddr.reset(new ddynamic_reconfigure::DDynamicReconfigure(pnh));

    // ddr->registerVariable<bool>("failsafe", &this->failsafe_);
    ddr->registerVariable<double>("failsafe_threshold_dist", &this->failsafe_threshold_dist_, "failsafe_threshold_dist", 0.0, 10.0);
    ddr->registerVariable<double>("failsafe_threshold_percentage", &this->failsafe_threshold_percentage_, "failsafe_threshold_percentage", 0.0, 1.0);
    ddr->registerVariable<double>("failsafe_timeout", &this->failsafe_timeout_, "failsafe_timeout", 0.0, 10.0);

    ddr->registerVariable<double>("cliff_threshold_constant", &this->cliff_thresh_constant_, "cliff_threshold_constant", 0.0, 0.5);
    ddr->registerVariable<int>("min_cliff_pts_in_seg", &this->min_cliff_pts_in_seg_, "min_cliff_pts_in_seg", 0, 100);

    ddr->publishServicesTopics();

    first_scan_cb_ = true;
    failsafe_activated_ = false;

}

void CliffDetector::initPubSubSrv(){
    //Subscribers
    scan_sub_ = nh.subscribe(scan_input_, 1000, &CliffDetector::cliffScanCB, this);
    //Publishers
    scan_cliff_pub_ = nh.advertise<sensor_msgs::LaserScan>(scan_output_, 1000);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel_safety", 10);
    scan_thresh_pub = nh.advertise<sensor_msgs::LaserScan>("/cliff_thresh_values", 1000);
}

void CliffDetector::cmd_velStop(){
    geometry_msgs::Twist cmd_vel_msg;
    //publish empty message (i.e. stop moving)
    cmd_vel_pub_.publish(cmd_vel_msg);
}


std::vector<float> CliffDetector::generateCliffLookup(int scan_size){
    std::vector<float> thresh_laser_scan;
    int min_idx, max_idx;

    if (scan_size == 0){
        ROS_ERROR("cliff_detector: Input scan message is empty!");
        return thresh_laser_scan;
    }

    double lidar_pitch_rad = lidar_pitch_deg_ * DEG2RAD; //convert to radians

    //distance travelled by ray from lidar to the ground.
    double middle_laser_length = lidar_height_ / sin(lidar_pitch_rad) ;

    if (scan_size % 2 == 0){
        min_idx = scan_size/2 ;
        max_idx = scan_size/2 ;
    }
    else{
        //we assume that for odd scan sizes, the 
        //number of laser scans on the one side 
        //of the x axis is higher than the other side
        min_idx = scan_size/2 ;
        max_idx = scan_size/2 +1;
    }

    //generate threshold values for each laser range
    for (int i = max_idx ; i >= -min_idx ; i--){
        //skip the middle beam along x axis if it doesn't exist on the lidar
        if (i == 0){
            continue;
        }
        //If there is no beam travelling down the X axis of the lidar,
        //then we need to offset all the beams
        // thresh_laser_scan.push_back(middle_laser_length/cos( (i * lidar_resolution_ - (lidar_resolution_/2)) * DEG2RAD));

        //without offset
        thresh_laser_scan.push_back(middle_laser_length/ cos( (i * lidar_resolution_) * DEG2RAD) + cliff_thresh_constant_);
    }

    failsafe_max_count_ = scan_size * failsafe_threshold_percentage_;

    return thresh_laser_scan;
}

void CliffDetector::detectCliff(sensor_msgs::LaserScan& _scan, std::vector<float>& _lookup_thresh_range){
    int cliff_pts_in_seg = 0; //No. cliff points in a segment
    int failsafe_count = 0; //No. failsafe points

    for (int i=0; i < _scan.ranges.size(); i++ ) {
        
        if (_scan.ranges[i] > (_lookup_thresh_range[i] + cliff_thresh_constant_)){
            _scan.ranges[i] = _lookup_thresh_range[i];
            cliff_pts_in_seg++;
        }
        
        else{
            //IF scan is below failsafe distance
            if (_scan.ranges[i] < failsafe_threshold_dist_ ){
                failsafe_count++;
            }

            _scan.ranges[i] = std::numeric_limits<float>::infinity();

            //IF segment size is less than min_cliff_pts_in_seg,
            //set the current segment to non-cliff points 
            if (cliff_pts_in_seg > 0 && cliff_pts_in_seg < min_cliff_pts_in_seg_){
                for (int j = i; j >= i-cliff_pts_in_seg; j--){
                    _scan.ranges[j] = std::numeric_limits<float>::infinity();
                }
            }

            //reset number of cliff points in a segment
            cliff_pts_in_seg = 0;
        }
    }

    //IF failsafe is activated, check if the lidar is blocked by something
    if (failsafe_){
        //get time difference between current time and the time elapsed
        //since the lidar failsafe wasn't activated
        double time_failsafe = ros::Time::now().toSec() - time_elapsed.toSec();

        if (failsafe_count >= failsafe_max_count_){
            //IF failsafe timeout exceeded, stop robot
            if (time_failsafe > failsafe_timeout_){
                ROS_ERROR("Cliff lidar is currently obstructed, robot has been stopped. Please check the sensors.");
                //TODO: Publish error code
                cmd_velStop();
                failsafe_activated_ = true;
            }
        }
        else{
            time_elapsed = ros::Time::now();
            failsafe_activated_ = false;
        }
    }
}

void CliffDetector::cliffScanCB(const sensor_msgs::LaserScan::ConstPtr& scan_in){
    scan_out = *scan_in;
    scan_thresh = *scan_in;

    scan_out.header.frame_id = "base_link";

    //generate lookup table once
    if (first_scan_cb_){
        //generate lookup table for expected laser ranges if cliff lidar sees a flat planar ground
        lookup_thresh_range_ = generateCliffLookup(scan_in->ranges.size());
        ROS_INFO("Lookup thresh range: %ld, scan input size: %ld",lookup_thresh_range_.size(), scan_in->ranges.size() );
        first_scan_cb_ = false;
    }
    
    detectCliff(scan_out, lookup_thresh_range_);

    //Generate a scan threshold message as a visual indication of the ranges (which when exceeded) 
    //that considered as negative obstacles 
    for (int i=0; i < scan_thresh.ranges.size(); i++ ) {
        scan_thresh.ranges[i] = lookup_thresh_range_[i];
    }

    scan_thresh_pub.publish(scan_thresh); //for debugging purposes, please comment out in production
    scan_cliff_pub_.publish(scan_out);
}

