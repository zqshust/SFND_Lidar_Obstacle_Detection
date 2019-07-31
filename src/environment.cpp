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
    bool renderScene = false; //true;

    bool render_ray = false;
    bool render_PointCloud = false;
    
    bool render_obst = false;
    bool render_plane = false;

    bool render_clusters = true;
    bool render_box = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //Generate this PCL point cloud, PCL::pointXYZ pointer, 
    if(render_ray)
        renderRays(viewer,lidar->position, inputCloud);
    if(render_PointCloud)
        renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    //Instantiate on the stack
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    //Create point cloud processor, Instantiate on the heap
    //ProcessPointClouds<pcl::PointXYZ> pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    //ProcessPointClouds<pcl::PointXYZI> pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    

    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud,100,0.2);
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud,100,0.2);

    //Ransac plane with newly created RansacPlane function, use plane and point formulas.
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.RansacPlane(inputCloud,100,0.25);
    //when distanceThreshold is set to 0.2 there will case that plane point are segment into obstacle point, finally I choose o.25, It's OK when I try it.

    if(render_obst)
        renderPointCloud(viewer, segmentCloud.first, "obsCloud", Color(1,0,0));
    if(render_plane)
        renderPointCloud(viewer, segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 2.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters){
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        }
        

        if(render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);

        }
        
        ++clusterId;
    }

  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

  inputCloud = pointProcessorI->FilterCloud(inputCloud,0.3,Eigen::Vector4f(-10,-6,-3,1),Eigen::Vector4f(30,6,4,1));

  //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud,100,0.2);


  //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  //renderPointCloud(viewer,segmentCloud.second,"oplaneCloud",Color(0,1,0));

  /*
  int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters){
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        }
        

        if(render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);

        }
        
        ++clusterId;
    }
    */
  renderPointCloud(viewer,inputCloud,"inputCloud");
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

    //simpleHighway(viewer);
    cityBlock(viewer);

    //Create point cloud processor
    /*
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1")
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    */

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}