#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <string>     
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sstream>	//使用stringstream需要引入这个头文件


using namespace std;


//模板函数：将string类型变量转换为常用的数值类型（此方法具有普遍适用性）
template <class Type>
Type stringToNum(const string& str)
{
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;    
}

int fileNameFilter(const struct dirent *cur) {
    std::string str(cur->d_name);
    if (str.find(".pcd") != std::string::npos) {
        return 1;
    }
    return 0;
}

std::vector<std::string> getDirBinsSortedPath(std::string dirPath) {
    struct dirent **namelist;
    std::vector<std::string> ret;
    int n = scandir(dirPath.c_str(), &namelist, fileNameFilter, alphasort);
    if (n < 0) {
        return ret;
    }
    for (int i = 0; i < n; ++i) {
        std::string filePath(namelist[i]->d_name);
        ret.push_back(filePath);
        free(namelist[i]);
    };
    free(namelist);
    return ret;
}


sensor_msgs::PointCloud2 read_pcd_path(string path, int num)
{
  vector<string> files;
  files = getDirBinsSortedPath(path.c_str());

  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  string path_final = path + files[num];
  cout << "Read file:" << path_final << endl; 
  pcl::io::loadPCDFile (path_final, cloud); //修改自己pcd文件所在路径
  //Convert the cloud to ROS message
  
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "vlp16";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer    
//！！！这一步需要注意，是后面rviz的 fixed_frame  !!!敲黑板，画重点。
  return output;
}




int main (int argc, char **argv)
{
  ros::init (argc, argv, "UandBdetect");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 10);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  string path = "/home/thinking/Downloads/bag/0216/02161708/";
  string bagname = "/home/thinking/Downloads/bag/0216/huawei_02161708.bag";

  ofstream fileout(bagname,ios::trunc);//ios::trunc是清除原文件内容,可不写,默认就是它
  if(!fileout){
      cout << "Create file failure...\n";
      exit(0);
  }
  fileout.close();

  vector<string> files;
  files = getDirBinsSortedPath(path.c_str());
  
  rosbag::Bag bag;
  double time = ros::Time::now().toSec();
  bag.open(bagname,rosbag::bagmode::Write);


  for (int num = 0 ; num < 1430; num++)
  {
    cout << fixed << setprecision(6) << stringToNum<double>(files[num].substr(0, 10) + "." + files[num].substr(10, 16)) << endl;
    output = read_pcd_path(path, num);
    output.header.frame_id = "vlp16" ;
    output.header.stamp = ros::Time().fromSec(stringToNum<double>(files[num].substr(0, 10) + "." + files[num].substr(10, 16)));
    // cout << "scan time:" << float(num/10)+time << endl; 
    cout << "scan time:" << output.header.stamp << endl; 
    bag.write("/velodyne_points",output.header.stamp,output);
  }
  // bag.close();
  // cout<<"Bag file write finish"<<endl;

  // // ros::Rate loop_rate(1);
  // // while (ros::ok())
  // // {
  // //   pcl_pub.publish(output);
  // //   ros::spinOnce();
  // //   loop_rate.sleep();
  // // }
  return 0;
}


