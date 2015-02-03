#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>

#include <unknown_objects_segmentation/SegmenterLight.h>

#define ROS_GREEN_STREAM(x) ROS_INFO_STREAM("\033[1;32m" << x << "\033[0m")
#define ROS_RED_STREAM(x) ROS_INFO_STREAM("\033[1;31m" << x << "\033[0m")
#define ROS_BLUE_STREAM(x) ROS_INFO_STREAM("\033[1;34m" << x << "\033[0m")
#define ROS_YELLOW_STREAM(x) ROS_INFO_STREAM("\033[1;33m" << x << "\033[0m")
#define ROS_PINK_STREAM(x) ROS_INFO_STREAM("\033[1;35m" << x << "\033[0m")
#define ROS_CYAN_STREAM(x) ROS_INFO_STREAM("\033[1;36m" << x << "\033[0m")

class SegmenterNode
{
protected:
  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  ros::Publisher _pub;

  boost::shared_ptr<segment::SegmenterLight> segmenter;

  // parameters
  int _levels;
  double _maxHeight;
  double _minHeight;
  double _maxRange;
  int _desiredNumPoints;

public:

  SegmenterNode()
    : _nh("~")
  {
//    _sub = _nh.subscribe("cloud_in", 10, &SegmenterNode::cloudCallback, this);
    _sub = _nh.subscribe("/camera/depth_registered/points", 1, &SegmenterNode::cloudCallback, this);
    _pub = _nh.advertise<sensor_msgs::PointCloud2>("extrudedCloud",10);

//    std::string model_path;
//    if(!_nh.getParam("model_path", model_path))
//    {
//        ROS_ERROR("MODEL PATH EMPTY");
//    }

    segmenter.reset(new segment::SegmenterLight("/home/bencemagyar/tiago_ws/src/unknown_objects_segmentation/config/model/"));
  }

protected:

  void cloudCallback(const sensor_msgs::PointCloud2& cloud)
  {
    ros::Time timeStart(ros::Time::now());

    pcl::PointCloud<pcl::PointXYZRGB> cloud_to_segment;
    pcl::fromROSMsg(cloud, cloud_to_segment);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_segment_ptr(
                new pcl::PointCloud<pcl::PointXYZRGB>(cloud_to_segment));

    // and now, MAGIC!!
    ROS_CYAN_STREAM("Now calling processPointCloud");
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr labeled_cloud_ptr =
            segmenter->processPointCloud(cloud_to_segment_ptr);

    pcl::io::savePCDFileASCII("test_pcd.pcd", *labeled_cloud_ptr);

    if(_pub.getNumSubscribers())
    {
        if(labeled_cloud_ptr->empty())
        {
            ROS_PINK_STREAM("Labeled cloud is empty, won't publish.");
            return;
        }
        ROS_YELLOW_STREAM("Publish segmented cloud...");
        sensor_msgs::PointCloud2 cloud_out;
        cloud_out.header.frame_id = cloud.header.frame_id;
        pcl::toROSMsg(*labeled_cloud_ptr, cloud_out);
        _pub.publish(cloud_out);
    }

    ROS_INFO_STREAM("Callback runtime (s): " <<
                    ros::Duration(ros::Time::now() - timeStart));
  }

};


int main (int argc, char *argv[])
{
  ros::init(argc, argv, "segmenter_node");
  ros::Time::init();
  ros::Rate rate(10);

  SegmenterNode node;

  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}


