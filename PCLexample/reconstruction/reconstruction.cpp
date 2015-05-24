#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/grid_projection.h>

using namespace pcl;
using namespace std;

int
main(int argc, char** argv)
{
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	if (io::loadPLYFile<PointXYZ>(argv[1], *cloud) == -1){
		cout << "fail" << endl;

	}
	else {

		cout << "loaded" << endl;

		cout << "begin passthrough filter" << endl;
		PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
		PassThrough<PointXYZ> filter;
		filter.setInputCloud(cloud);
		filter.filter(*filtered);
		cout << "passthrough filter complete" << endl;

		cout << "begin normal estimation" << endl;
		NormalEstimationOMP<PointXYZ, Normal> ne;
		ne.setNumberOfThreads(8);
		ne.setInputCloud(filtered);
		ne.setRadiusSearch(0.01);
		Eigen::Vector4f centroid;
		compute3DCentroid(*filtered, centroid);
		ne.setViewPoint(centroid[0], centroid[1], centroid[2]-100);

		PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
		ne.compute(*cloud_normals);
		cout << "normal estimation complete" << endl;
		//cout << "reverse normals' direction" << endl;

		//for (size_t i = 0; i < cloud_normals->size(); ++i){
		//	cloud_normals->points[i].normal_x *= -1;
		//	cloud_normals->points[i].normal_y *= -1;
		//	cloud_normals->points[i].normal_z *= -1;
		//}

		cout << "combine points and normals" << endl;
		PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
		concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

		PolygonMesh mesh;
#if 1
		cout << "begin poisson reconstruction" << endl;
		Poisson<PointNormal> poisson;
		poisson.setDepth(8);
		//poisson.setIsoDivide(8);
		//poisson.setSolverDivide(8);
		//poisson.setPointWeight(1);
		poisson.setInputCloud(cloud_smoothed_normals);
		poisson.reconstruct(mesh);
#elif 0
		cout << "begin marching cube reconstruction" << endl;
		MarchingCubesRBF<PointNormal> mc;
		mc.setInputCloud(cloud_smoothed_normals);
		mc.reconstruct(mesh);
#else
		cout << "begin GridProjection reconstruction" << endl;
		GridProjection<PointNormal> gp;
		gp.setMaxBinarySearchLevel(6);
		gp.setNearestNeighborNum(4);
		gp.setResolution(1024);
		gp.setInputCloud(cloud_smoothed_normals);
		gp.reconstruct(mesh);
#endif

		io::savePLYFile(argv[2], mesh);

	}
	return (0);
}