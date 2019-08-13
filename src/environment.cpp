/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan1 = lidar->scan();
    // renderRays(viewer, lidar->position, scan1);
    // renderPointCloud(viewer, scan1, "Point Cloud", Color(0, 1, 1));
    // TODO:: Create point processor
    // ProcessPointClouds<pcl::PointXYZ> pointProcessor; // instantiates on a stack
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); //another way to instantiate pointProcessor. instantiates on a heap
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(scan1, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstacles", Color(1, 0, 0));
    // renderPointCloud(viewer, segmentCloud.second, "plane", Color(0, 1, 0));

    // uncomment this section to use the ransac method from the quiz
    /*std::unordered_set<int> inliers = pointProcessor->Ransac(scan1, 100, 0.2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  	for(int index = 0; index < scan1->points.size(); index++)
  	{
  		pcl::PointXYZ point = scan1->points[index];
  		if(inliers.count(index))
  			cloudInliers->points.push_back(point);
  		else
  			cloudOutliers->points.push_back(point);
  	}
  	// Render 2D point cloud with inliers and outliers
  	if(inliers.size())
  	{
  		  renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
    		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
  	}
  	else
  	{
  		renderPointCloud(viewer,scan1,"data");
  	}*/

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
      std::cout << "cluster size ";
      pointProcessor->numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
      ++clusterId;
      Box boundingBox = pointProcessor->BoundingBox(cluster);
      renderBox(viewer, boundingBox, clusterId);
    }

}

// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>(); // instantiates on a heap
  // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  // renderPointCloud(viewer, inputCloud, "input Cloud");
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25, Eigen::Vector4f(-18, -5, -5, 1), Eigen::Vector4f(18, 6, 10, 1));
  // renderPointCloud(viewer, filterCloud, "Filtered Cloud");
  std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloudI = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
  // renderPointCloud(viewer, segmentCloudI.first, "obstacle", Color(1, 0, 0));
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusterI = pointProcessorI->Clustering(segmentCloudI.first, 0.75, 12, 300);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusterI) {
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstacle cloud"+std::to_string(clusterId), colors[clusterId]);
    ++clusterId;
    Box boundingBoxI = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, boundingBoxI, clusterId);
  }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    // simpleHighway(viewer);
    // cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        inputCloud = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloud);
        streamIterator++;
        if (streamIterator == stream.end()) {
          streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    }
}
