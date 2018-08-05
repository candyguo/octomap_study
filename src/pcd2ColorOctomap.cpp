//
// Created by ccfy on 18-8-4.
//
#include <iostream>
#include <assert.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
using namespace std;

int main(int argc,char** argv)
{
	if(argc != 3)
	{
		cout<<"Usage: "<<"pcd2ColorOctomap <input_files> <output_file>"<<endl;
	}
	string input_file = argv[1];
	string output_file = argv[2];
	pcl::PointCloud<pcl::PointXYZRGBA> cloud;
	pcl::io::loadPCDFile(input_file,cloud);
	cout<<"point cloud size = "<<cloud.points.size()<<endl;
	cout<<"copying data into octomap..."<<endl;
	octomap::ColorOcTree tree(0.05);
	for(auto point : cloud.points)
	{
		tree.updateNode(octomap::point3d(point.x,point.y,point.z),true);
	}
	for(auto point : cloud.points)
	{
		tree.integrateNodeColor(point.x,point.y,point.z,point.r,point.g,point.b);
	}
	tree.updateInnerOccupancy();
	tree.writeBinary(output_file);
	return 0;
}