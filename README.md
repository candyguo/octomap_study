# octomap_study
octomap : convert point cloud pcd to ogm

keyfunction: 
tree.updateNode(octomap::point3d(point.x,point.y,point.z),true);
tree.updateInnerOccupancy();
tree.insertPointCloud(cloud_octo,octomap::point3d(poses[i](0,3),poses[i](1,3),poses[i](2,3)));
tree.integrateNodeColor(point.x,point.y,point.z,point.r,point.g,point.b);
             
