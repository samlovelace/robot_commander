
#include "PointCloudHandler.h"

PointCloudHandler::PointCloudHandler()
{

}

PointCloudHandler::~PointCloudHandler()
{

}

bool PointCloudHandler::fromFile(const std::string& aFilePath, sensor_msgs::msg::PointCloud2& aCloudOut)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; 
    if(pcl::io::loadPLYFile<pcl::PointXYZ>(aFilePath, *cloud) == -1)
    {
        std::cerr << "Failed to load object point cloud from " << aFilePath; 
        return false; 
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud, pcl_pc2);

    pcl_conversions::fromPCL(pcl_pc2, aCloudOut);
    aCloudOut.header.frame_id = "map";

    return true; 
}