//
// Created by ccfy on 18-8-4.
//
#include <iostream>
#include <assert.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
using namespace std;

int main(int argc,char** argv)
{
	if(argc != 3)
	{
		cout<<"Usage: pcd2octomap <input_file> <output file>"<<endl;
		return -1;
	}
	string input_file = argv[1], output_file = argv[2];
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::io::loadPCDFile(input_file,cloud);
	cout<<"begin octomap generate process..."<<endl;
	cout<<"point cloud size = "<<cloud.points.size()<<endl;
	cout<<"copy point cloud into octomap..."<<endl;
	octomap::OcTree tree(0.05);
	for(auto point : cloud.points)
	{
		tree.updateNode(octomap::point3d(point.x,point.y,point.z),true);
	}
	//update octomap
	tree.updateInnerOccupancy();
	tree.writeBinary(output_file);
	cout<<"done"<<endl;
	return 0;
}


