#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include"pcl/visualization/pcl_visualizer.h"
#include"pcl/search/kdtree.h"
#include"pcl/features/normal_3d_omp.h"
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

using namespace std;

// 定义全局变量 法向量
extern pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

// 提取点云子集
vector<int> get_subset(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int points_num, int start_position)
{
    vector<int> point_indices(points_num);
    for(size_t i = 0; i < points_num; ++i)
    {
        point_indices[i] = i + start_position;
    }
    cout << "点云总数为：" << cloud->points.size() << endl;
    cout << "处理点云数为：" << points_num << endl;

    return point_indices;
}

// 可视化
void visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   vector<int> point_indices,
                   vector<int> marker)
{
    //----------为方便可视化，将前10%点云提出-------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*cloud, point_indices, *cloud1);
    pcl::copyPointCloud(*cloud, marker, *cloud2);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normal viewer"));

    // 设置背景颜色
    viewer->setBackgroundColor(255, 255, 255);
    viewer->addText("bz12",10,10,"text");
    // 设置点云颜色
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud1,0,225,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud,225,0,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2,0,0,225);

    // 添加坐标系
    viewer->addCoordinateSystem(50.0);

    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
    // viewer->addPointCloud<pcl::PointXYZ> (cloud1, single_color1, "sample cloud1");
    viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color2, "sample cloud2");

    // 添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，20表示需要显示法向的点云间隔，即20个点显示一次法向，1表示法向长度
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud2, normals, 20, 1, "normals");
    
    // 设置点云大小
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud2");

    while (!viewer->wasStopped()) //添加while空循环之后，运行结果弹出并保持PCL Viewer窗口
    {    
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

// 计算法向量
void calculate_norms(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     pcl::IndicesPtr indices)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n; //OMP加速
    n.setInputCloud(cloud);
    n.setIndices(indices);
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());  // 建立kdtree来进行近邻点集搜索
    n.setSearchMethod(tree);
    //n.setRadiusSearch(0.01);
    n.setKSearch(15);             //点云计算时，需要搜索的近邻点个数
    
    n.setNumberOfThreads(10);   
    n.compute(*normals);         //开始计算法向量

    pcl::io::savePCDFileASCII("/home/jz/map/bz12_normals.pcd", *normals);  // 保存法向量信息
}

vector<int> analyze(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int start_position)
{
    // 退化方向
    Eigen::Vector3d degeneration(-0.9962,-0.08715,0.0);

    vector<int> marker;

    // 利用垂直特征和曲率筛选出凹凸点
    for(size_t i = 0; i < normals->points.size(); ++i)
    {
        Eigen::Vector3d norm(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
        
        if(abs(norm.dot(degeneration)) > 0.97 && normals->points[i].curvature < 0.08)
        {
            marker.push_back(i + start_position);
            //marker.push_back(i);
        }        
    }

    cout << "符合条件的点数：" << marker.size() << endl; 

    ofstream f("/home/jz/map/marker.txt", ios::app);
    for (int j = 0; j < marker.size(); ++j) 
    {
        f << marker[j] << "\n";
    }

    // 得到点的坐标
    Eigen::MatrixXf marker_location(marker.size(),3); 
    for(int j = 0; j < marker.size(); ++j)
    {
        marker_location(j,0) = cloud->points[marker[j]].x;
        marker_location(j,1) = cloud->points[marker[j]].y;
        marker_location(j,2) = cloud->points[marker[j]].z;
    }



    return marker;

}

int main()
{

    //-----------------加载点云数据--------------------
    //定义指针类型变量cloud并开辟空间进行初始化
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jz/map/bz12/pcd_frames/map.pcd",*cloud) == -1)
    {
        PCL_ERROR("Could not read file\n");
    }

    //---------------提取点云子集------------------
    vector<int> point_indices;
    int points_num, start_position;
    points_num = floor(cloud->points.size() * 0.3);        // 子集个数
    start_position = floor(cloud->points.size() * 0.4);    // 子集起始位置

    point_indices = get_subset(cloud, points_num, start_position);

    //-----------------传递索引--------------------
    pcl::IndicesPtr indices(new vector <int>(point_indices));

    //-----------------计算法线--------------------
    calculate_norms(cloud,indices);

    // 分析
    vector<int> marker;
    marker = analyze(cloud, start_position);

    //-----------------可视化--------------------
    visualization(cloud, point_indices, marker);

    


    return 0;
}
