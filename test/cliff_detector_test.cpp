#include <gtest/gtest.h>

#include "cliff_detector/cliff_detector.h"

#define PI 3.141592653589793
#define DEG2RAD 0.0174532

//Test naming conventions
// Test suites are CamelCased, like C++ types
// Test cases are camelCased, like C++ functions

//ASSERT_* versions generate fatal failures when they fail, and abort the current function.
//EXPECT_* versions generate nonfatal failures, which don’t abort the current function.
//*_EQ : EQUAL
//*_NE : NOT EQUAL

//physical lidar properties
double lidar_pitch_deg_ = 30.0;
double lidar_height_ = 0.925;
double lidar_resolution_ = 1.0;
double lidar_frequency_ = 14.5;

double failsafe_threshold_dist_ = 0.2;       //anything less than this is a failsafe point
double failsafe_threshold_percentage_ = 0.2; //anything less than this is a failsafe point
double cliff_thresh_constant_ = 0.001;       //anything less than this is a failsafe point
int min_cliff_pts_in_seg_ = 5;

//helper to create laserscan message
auto createLaserScanMsg = [](int scan_size, double _failsafe_thresh_dist,
                             double failsafe_pts_size, double cliff_pts_size)
{
  //initialize data structs
  double ranges[scan_size];
  double intensities[scan_size];
  std::vector<float> lookup_thresh_range;

  ros::Time scan_time = ros::Time::now();

  //Create scan message using SICK TIM 240 technical data
  sensor_msgs::LaserScan scan;
  scan.header.stamp = scan_time;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = -50 * DEG2RAD;
  scan.angle_max = 50 * DEG2RAD;
  scan.angle_increment = 1 * DEG2RAD;
  scan.time_increment = (1 / lidar_frequency_) / (scan_size);
  scan.range_min = 0.05;
  scan.range_max = 10.0;

  scan.ranges.resize(scan_size);
  scan.intensities.resize(scan_size);

  //generate threshold values for each laser range
  CliffDetector cliff_detector;
  cliff_detector.setLidarParams(lidar_pitch_deg_, lidar_height_, lidar_resolution_);
  lookup_thresh_range = cliff_detector.generateCliffLookup(scan_size);

  //assign non-thresholded values to scan message
  for (int i = 0; i < scan_size; ++i)
  {
    scan.ranges[i] = lookup_thresh_range[i] - 0.01;
    scan.intensities[i] = 0;
  }

  //add failsafe points
  for (unsigned int i = 0; i < failsafe_pts_size; ++i)
  {
    scan.ranges[i] = _failsafe_thresh_dist - 0.01;
  }

  //add cliff points
  for (unsigned int i = failsafe_pts_size; i < failsafe_pts_size + cliff_pts_size; ++i)
  {
    scan.ranges[i] = lookup_thresh_range[i] + 0.01;
  }

  return scan;
};

class CliffDetectorFixture : public ::testing::Test
{
protected:
  CliffDetectorFixture()
  {
    // initialization code here
  }

  void SetUp() override
  {
    // code here will execute just before the test ensues
    normal_scan = createLaserScanMsg(140, failsafe_threshold_dist_, 0, 0);
    normal_scan_noisy = createLaserScanMsg(140, failsafe_threshold_dist_, 4, 4);
    failsafe_scan = createLaserScanMsg(140, failsafe_threshold_dist_, 75, 0);
    cliff_scan = createLaserScanMsg(140, failsafe_threshold_dist_, 0, 6);
  }

  /* If you need to cleanup the resources any tests are using, you can do it in TearDown(). */
  void TearDown() override
  {
    // code here will be called just after the test completes
    // ok to through exceptions from here if need be
  }

  sensor_msgs::LaserScan normal_scan;       //contains perfect scan points
  sensor_msgs::LaserScan normal_scan_noisy; //contains some cliff points and failsafe points (but not exceeding the segment threshold)
  sensor_msgs::LaserScan failsafe_scan;     //contains enough failsafe points to activate failsafe
  sensor_msgs::LaserScan cliff_scan;        //contains enough cliff points to exceed the segment threshold
};

TEST_F(CliffDetectorFixture, TestLookupValues)
{
  CliffDetector cliff_detector;
  cliff_detector.setLidarParams(lidar_pitch_deg_, lidar_height_, lidar_resolution_);
  cliff_detector.setFailsafeParams(true, failsafe_threshold_dist_, failsafe_threshold_percentage_, 0.000000001);
  cliff_detector.setCliffParams(cliff_thresh_constant_, min_cliff_pts_in_seg_);

  int scan_size1 = 20, scan_size2 = 21, scan_size3 = 90;
  std::vector<float> lookup_thresh_range1, lookup_thresh_range2, lookup_thresh_range3;

  //TEST1: scan_size == lookup_thresh size
  lookup_thresh_range1 = cliff_detector.generateCliffLookup(scan_size1);
  ASSERT_EQ(scan_size1, lookup_thresh_range1.size());

  lookup_thresh_range2 = cliff_detector.generateCliffLookup(scan_size2);
  ASSERT_EQ(scan_size2, lookup_thresh_range2.size());

  lookup_thresh_range3 = cliff_detector.generateCliffLookup(scan_size3);
  ASSERT_EQ(scan_size3, lookup_thresh_range3.size());

  for (int i = 0; i < lookup_thresh_range3.size() / 2; i++)
  {

    //TEST2: lookup_thresh_range[i] = lookup_thresh_range[j], where j = lookup_thresh_range.size()-i
    ASSERT_EQ(lookup_thresh_range3[i], lookup_thresh_range3[lookup_thresh_range3.size() - i - 1]);

    //TEST3: All values in lookup_thresh_range are positive.
    ASSERT_TRUE((lookup_thresh_range3[i] > 0) && (lookup_thresh_range3[lookup_thresh_range3.size() - i - 1] > 0));
  }
}

//TOTEST Failsafe timeout: When all laser ranges are below
//failsave_threshold_dist_, cmd_vel_stop is activated

TEST_F(CliffDetectorFixture, TestFailsafe)
{
  CliffDetector cliff_detector;
  cliff_detector.setLidarParams(lidar_pitch_deg_, lidar_height_, lidar_resolution_);
  cliff_detector.setFailsafeParams(true, failsafe_threshold_dist_, failsafe_threshold_percentage_, 0.000000001);
  cliff_detector.setCliffParams(cliff_thresh_constant_, min_cliff_pts_in_seg_);

  int scan_size = 140;
  std::vector<float> lookup_thresh_range;

  lookup_thresh_range = cliff_detector.generateCliffLookup(scan_size);

  //TEST1: Failsafe should not be activated for normal_scan
  cliff_detector.detectCliff(normal_scan, lookup_thresh_range);
  ASSERT_FALSE(cliff_detector.failsafe_activated_);

  //TEST2: Failsafe should not be activated for normal_scan_noisy
  cliff_detector.detectCliff(normal_scan_noisy, lookup_thresh_range);
  ASSERT_FALSE(cliff_detector.failsafe_activated_);

  //TEST3: Failsafe should be activated for failsafe_scan
  cliff_detector.detectCliff(failsafe_scan, lookup_thresh_range);
  ASSERT_TRUE(cliff_detector.failsafe_activated_);
}

TEST_F(CliffDetectorFixture, TestCliffPoints)
{
  CliffDetector cliff_detector;
  cliff_detector.setLidarParams(lidar_pitch_deg_, lidar_height_, lidar_resolution_);
  cliff_detector.setFailsafeParams(true, failsafe_threshold_dist_, failsafe_threshold_percentage_, 0.000000001);
  cliff_detector.setCliffParams(cliff_thresh_constant_, min_cliff_pts_in_seg_);

  int scan_size = 140;
  std::vector<float> lookup_thresh_range;

  lookup_thresh_range = cliff_detector.generateCliffLookup(scan_size);

  //TEST1: normal_scan should have 0 cliff points
  cliff_detector.detectCliff(normal_scan, lookup_thresh_range);

  int num_cliff_points = 0;
  for (int i = 0; i < normal_scan.ranges.size(); ++i)
  {
    //get number of cliff points
    if (normal_scan.ranges[i] == lookup_thresh_range[i])
    {
      num_cliff_points++;
    }
  }
  ASSERT_EQ(num_cliff_points, 0);

  //TEST2: normal_scan_noisy should have 0 cliff points
  //since the number of cliff points < min_cliff_pts_in_seg_
  cliff_detector.detectCliff(normal_scan_noisy, lookup_thresh_range);

  num_cliff_points = 0;
  for (int i = 0; i < normal_scan_noisy.ranges.size(); ++i)
  {
    //get number of cliff points
    if (normal_scan_noisy.ranges[i] == lookup_thresh_range[i])
    {
      num_cliff_points++;
    }
  }
  ASSERT_EQ(num_cliff_points, 0);

  //TEST3: cliff_scan should have 6 cliff points
  //since the number of cliff points < min_cliff_pts_in_seg_
  cliff_detector.detectCliff(cliff_scan, lookup_thresh_range);

  num_cliff_points = 0;

  for (int i = 0; i < cliff_scan.ranges.size(); ++i)
  {
    //get number of cliff points
    if (cliff_scan.ranges[i] == lookup_thresh_range[i])
    {
      num_cliff_points++;
    }
  }
  ASSERT_EQ(num_cliff_points, 6);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cliff_detector_tester");
  ros::NodeHandle nh; //initialize if using ROS

  return RUN_ALL_TESTS(); //Returns 0 if tests are successful, otherwise return 1. MUST BE RETURNED! And called only ONCE!
}