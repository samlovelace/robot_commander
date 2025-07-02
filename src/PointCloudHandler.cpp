
#include "PointCloudHandler.h"

PointCloudHandler::PointCloudHandler()
{

}

PointCloudHandler::~PointCloudHandler()
{

}

bool PointCloudHandler::fromFile(const std::string& aFilePath, sensor_msgs::msg::PointCloud2& aCloudOut)
{
    // Define the point cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read the PLY file
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(aFilePath, *cloud) == -1)
    {
        std::cerr << "Couldn't read file: " << aFilePath << std::endl;
        return -1;
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud, pcl_pc2);

    pcl_conversions::fromPCL(pcl_pc2, aCloudOut);
    aCloudOut.header.frame_id = "map";

    return true; 
}