#include <iostream>
#include<ros/ros.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/cloud_viewer.h>

#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Core>

#include <math.h>

using namespace std;

typedef pcl::PointXYZI PointType;
static double last_x, last_y, last_z, last_roll, last_yaw, last_pitch, last_t;
static double last_x2, last_y2, last_z2, last_roll2, last_yaw2, last_pitch2;
static int i = 0;


void filterCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, 
					ros::Publisher *pcl_pub)
{
	// 03. publish filtered pointcloud 
	pcl::PointCloud<PointType>::Ptr raw_cloud(new pcl::PointCloud<PointType>());
	pcl::fromROSMsg(*msg, *raw_cloud);

	// pcl::VoxelGrid<PointType> vg;   //滤波对象
	// pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);//创建保存体素滤波后的点对象cloudFiltered
	// vg.setInputCloud(raw_cloud);//输入cloud
	// vg.setLeafSize(0.2, 0.2, 0.2); // 设置叶子大小（这么大个叶子节点内只提取一个点）
	// vg.filter(*cloudFiltered);//滤波后的点云保存在cloudFiltered
							  
	//过滤掉在用户给定立方体内的点云数据
	//理解：将自身车辆作为坐标轴的中心点，然后在身边自身为中心 ，圈出范围，成为每一次运动时候的感兴趣区域，也就是只关心区域内点的聚类等后续操作
	pcl::PointCloud<PointType>::Ptr cloudRegion(new pcl::PointCloud<PointType>);
	pcl::CropBox<PointType> region(true);
	region.setMin(Eigen::Vector4f(-15, -15, 0, 1));
	region.setMax(Eigen::Vector4f(15, 15, 3, 1));
	region.setInputCloud(raw_cloud);
	region.filter(*cloudRegion);
 
	// //提取车身周围范围内的所有的点，并将提取到的所有点保存在indices中
	// std::vector<int> indices;
	// pcl::CropBox<pcl::PointXYZ> roof(true);
	// roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));//看数据像是车身大小
	// roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
	// roof.setInputCloud(cloudRegion);//输入的是上部中的立方体
	// roof.filter(indices);
	// pcl::PointIndices::Ptr inliers{ new pcl::PointIndices }; //创建一个内点对象，将提取到车身周围点，放到内点对象中
	// for (int point : indices) 
	// {
	// 	inliers->indices.push_back(point);
	// }
 
	// //创建点云提取函数
	// pcl::ExtractIndices<pcl::PointXYZ> extract;
	// extract.setInputCloud(cloudRegion);
	// extract.setIndices(inliers);
	// extract.setNegative(true);  //false 提取内点也就是提取车身周围的几个点，， true提取出了车身周围的点
	// extract.filter(*cloudRegion);
 
	// // /*pcl::PCDWriter writer;
	// // writer.write("C:\\Users\\Administrator\\Downloads\\求助\\求助\\tree-2-Rend.pcd", *cloud_filtered1);*/
 
	// // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("显示窗口"));  //窗口显示点云
	// // viewer->addPointCloud(cloudRegion, "*cloud");
	// // viewer->resetCamera();		//相机点重置
	// // viewer->spin();

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloudRegion,output);
    output.header.frame_id="os_sensor";
	pcl_pub->publish(output);
}

void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg, 
					  ros::Publisher *odom_pub, 
					  ros::Publisher *map_pub,
					  ros::Publisher *uv_pub,
                      ros::Publisher *gps_pub)
{
	// 01. publish base to odom
	static tf::TransformListener listener;
	static tf::StampedTransform transform;

	try
	{
		listener.waitForTransform("map", "base_footprint", msg->transforms[0].header.stamp, ros::Duration(0.1));
		listener.lookupTransform("map", "base_footprint", msg->transforms[0].header.stamp, transform);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	
	double x, y, z, roll, pitch, yaw;
	x = transform.getOrigin().x();
	y = transform.getOrigin().y();
	z = transform.getOrigin().z();


	// geometry_msgs::PoseStamped uv;
	// uv.header.stamp = msg->transforms[0].header.stamp;
	// uv.pose.position.x = (683.25 - y * 5) * 4;
	// uv.pose.position.y = (508.25 - x * 5) * 4;
	// uv_pub->publish(uv);

	Eigen::Quaterniond q;
	q.x() = transform.getRotation().getX();
	q.y() = transform.getRotation().getY();
	q.z() = transform.getRotation().getZ();
	q.w() = transform.getRotation().getW();


	nav_msgs::Odometry odometry;
	odometry.header.stamp = msg->transforms[0].header.stamp;
	odometry.header.frame_id = "map";
	odometry.child_frame_id = "vehicle";

	odometry.pose.pose.position.x = x;
	odometry.pose.pose.position.y = y;
	odometry.pose.pose.position.z = z;
	odometry.pose.pose.orientation.x = q.x();
	odometry.pose.pose.orientation.y = q.y();
	odometry.pose.pose.orientation.z = q.z();
	odometry.pose.pose.orientation.w = q.w();

	if(i != 0)
	{
		odometry.twist.twist.linear.x = (x - last_x) / (msg->transforms[0].header.stamp.toSec() - last_t);
		odometry.twist.twist.linear.y = (y - last_y) / (msg->transforms[0].header.stamp.toSec() - last_t);
		odometry.twist.twist.linear.z = (z - last_z) / (msg->transforms[0].header.stamp.toSec() - last_t);
		odometry.twist.twist.angular.x = (roll - last_roll) / (msg->transforms[0].header.stamp.toSec() - last_t);
		odometry.twist.twist.angular.y = (pitch - last_pitch) / (msg->transforms[0].header.stamp.toSec() - last_t);
		odometry.twist.twist.angular.z = (yaw - last_yaw) / (msg->transforms[0].header.stamp.toSec() - last_t);
	}

	odom_pub->publish(odometry);
	last_x = x; last_y = y; last_z = z;
	last_roll = roll; last_pitch = pitch; last_yaw = yaw;
	


	// 02. publish base to ecef
	static tf::TransformListener listener2;
	static tf::StampedTransform transform2;

	try
	{
		listener2.waitForTransform("ecef", "base_footprint", msg->transforms[0].header.stamp, ros::Duration(0.1));
		listener2.lookupTransform("ecef", "base_footprint", msg->transforms[0].header.stamp, transform2);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	
	double x2, y2, latitude, latitude_rad, longtitude, longtitude_rad;
	x2 = transform2.getOrigin().x();
	y2 = transform2.getOrigin().y();

    double a = 6378137.;  // semi-major axis, equator to center.
    double f = 1. / 298.257223563;
    double b = a * (1. - f);  // semi-minor axis, pole to center.
    double a_squared = a * a;
    double b_squared = b * b;
    double e_squared = (a_squared - b_squared) / a_squared;


    longtitude_rad = atan2(y2, x2);
    longtitude = longtitude_rad / M_PI * 180;


    latitude_rad = asin(sqrt((a_squared * sin(longtitude_rad) * sin(longtitude_rad) - y2 * y2)
                                / (a_squared * sin(longtitude_rad) * sin(longtitude_rad) + (f * f - 2 * f) * y2 * y2) ));
    latitude = latitude_rad / M_PI * 180;


    nav_msgs::Odometry gps_location;
	gps_location.header.stamp = msg->transforms[0].header.stamp;
	gps_location.header.frame_id = "ecef";
	gps_location.child_frame_id = "vehicle";

	gps_location.pose.pose.position.x = latitude;
	gps_location.pose.pose.position.y = longtitude;
	gps_location.pose.pose.position.z = 0;
	gps_location.pose.pose.orientation.x = 0;
	gps_location.pose.pose.orientation.y = 0;
	gps_location.pose.pose.orientation.z = 0;
	gps_location.pose.pose.orientation.w = 1;

    gps_location.twist.twist.linear.x = 0;
    gps_location.twist.twist.linear.y = 0;
    gps_location.twist.twist.linear.z = 0;
    gps_location.twist.twist.angular.x = 0;
    gps_location.twist.twist.angular.y = 0;
    gps_location.twist.twist.angular.z = 0;

    gps_pub->publish(gps_location);



	i = i + 1;

}
 
int main(int argc,char **argv)
{
	ros::init(argc, argv, "pointfilter");
	ros::NodeHandle nh;
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/base2odometry", 100);
	ros::Publisher map_pub = nh.advertise<nav_msgs::Odometry>("/base2map", 100);
    ros::Publisher gps_pub = nh.advertise<nav_msgs::Odometry>("/base2gps", 100);
	ros::Publisher uv_pub = nh.advertise<geometry_msgs::PoseStamped>("/uv_pixel", 100);
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointfilter_output",100);
	// ros::Subscriber sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>
	// 							("/robot_Z/pointcloud", 1000, boost::bind(&filterCallback, _1, &pcl_pub));
	ros::Subscriber sub_tf = nh.subscribe<tf2_msgs::TFMessage>
								("/tf", 1000, boost::bind(&tfCallback, _1, &odom_pub, &map_pub, &uv_pub, &gps_pub));
    
    ros::Rate loop_rate(1);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

	return (0);
}
