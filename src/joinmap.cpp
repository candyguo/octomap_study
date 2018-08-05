//
// Created by ccfy on 18-8-4.
//
#include <iostream>
#include <vector>
#include <assert.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/format.hpp>
using namespace std;

float camera_scale  = 1000;
float camera_cx     = 325.5;
float camera_cy     = 253.5;
float camera_fx     = 518.0;
float camera_fy     = 519.0;

int main(int argc,char** argv)
{
	ifstream fin("../data/keyframe.txt");
	vector<int>keyframes;
	int index_keyframe = 0;
	while(!fin.eof())
	{
		fin>>index_keyframe;
		keyframes.push_back(index_keyframe);
	}
	keyframes.pop_back();
	fin.close();
	for(auto i : keyframes)
	{
	    cout<<i<<endl;
	}
	cout<<"load total "<<keyframes.size()<<endl;

	//read pose vector
	vector<Eigen::Isometry3d> poses;
	fin.open("../data/trajectory.txt");
	double x,y,z;
	double qx,qy,qz,qw;
	while(!fin.eof())
	{
		fin>>index_keyframe>>x>>y>>z>>qx>>qy>>qz>>qw;
		Eigen::Vector3d translation(x,y,z);
		Eigen::Quaterniond quaternion(qw,qx,qy,qz);
		Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
		t.pretranslate(translation);
		t.rotate(quaternion);
		poses.push_back(t);
	}
	poses.pop_back();
	fin.close();
    cout<<"the size of poses is "<<poses.size()<<endl;
	octomap::ColorOcTree tree(0.05);
	for(int i=0;i<keyframes.size();i++)
	{
		pcl::PointCloud<pcl::PointXYZRGBA> cloud;
		cout<<"converting "<<i<<"th keyframe ..."<<endl;
		int k = keyframes[i];
		boost::format fmt("../data/rgb_index/%d.ppm");
		cv::Mat rgb = cv::imread((fmt%k).str().c_str());
		if(rgb.empty())
			cout<<"frame "<<k<<"rgb read failre"<<endl;
		fmt = boost::format("../data/dep_index/%d.pgm");
		cv::Mat depth = cv::imread((fmt%k).str().c_str());
        if(depth.empty())
			cout<<"frame "<<k<<"depth read failre"<<endl;
		for ( int m=0; m<depth.rows; m++ )
			for ( int n=0; n<depth.cols; n++ )
			{
				ushort d = depth.ptr<ushort> (m) [n];
				if (d == 0)
					continue;
				float z = float(d) / camera_scale;
				float x = (n - camera_cx) * z / camera_fx;
				float y = (m - camera_cy) * z / camera_fy;
				pcl::PointXYZRGBA p;
				p.x = x; p.y = y; p.z = z;

				uchar* rgbdata = &rgb.ptr<uchar>(m)[n*3];
				uchar b = rgbdata[0];
				uchar g = rgbdata[1];
				uchar r = rgbdata[2];

				p.r = r; p.g = g; p.b = b;
				cloud.points.push_back( p );
			}
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA>());
		pcl::transformPointCloud(cloud,*temp,poses[i].matrix());

		octomap::Pointcloud cloud_octo;
		for(auto point : temp->points)
		{
			cloud_octo.push_back(point.x,point.y,point.z);
		}

		tree.insertPointCloud(cloud_octo,octomap::point3d(poses[i](0,3),poses[i](1,3),poses[i](2,3)));
		for(auto point : temp->points)
		{
			tree.integrateNodeColor(point.x,point.y,point.z,point.r,point.g,point.b);
		}
	}
	tree.updateInnerOccupancy();
	tree.writeBinary("../data/map.bt");
	cout<<"done."<<endl;
	return 0;
}