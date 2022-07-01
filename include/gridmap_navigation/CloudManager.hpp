// TF
#include <geometry_msgs/TransformStamped.h>

// ROS Msg
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL Basic for ROS
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// pcl filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// pcl feature
#include <pcl/features/normal_3d.h>

template <typename pointType>
class CloudManager
{
private:
  /* data */
public:
  CloudManager();
  ~CloudManager() = default;

  bool transform(const typename pcl::PointCloud<pointType>::Ptr &cloudIn,
                 typename pcl::PointCloud<pointType>::Ptr &cloudOut,
                 const geometry_msgs::Transform &tf,
                 const std::string &targetFrame);
  void publish(const typename pcl::PointCloud<pointType>::Ptr &msg,
               const ros::Publisher &publisher);

  void passthroughFilter(typename pcl::PointCloud<pointType>::Ptr &cloud,
                         double minRange, double maxRange, double minZ,
                         double maxZ, double filterAngle = 0.0);
  void passthroughFilter(typename pcl::PointCloud<pointType>::Ptr &cloud,
                         const std::string &fieldName, double minVal,
                         double maxVal);

  void voxelFilter(typename pcl::PointCloud<pointType>::Ptr &cloud, float lx,
                   float ly, float lz);

  void
  computeNormals(typename pcl::PointCloud<pointType>::Ptr &cloudIn,
                 typename pcl::PointCloud<pcl::Normal>::Ptr &cloudNormalOut,
                 double searchRadius);
  void
  computeNormals(typename pcl::PointCloud<pointType>::Ptr &cloudIn,
                 typename pcl::PointCloud<pcl::Normal>::Ptr &cloudNormalOut,
                 float nearK);

  void pubNormalArrow(const pcl::PointXYZINormal &normal,
                      ros::Publisher &publisher, const std::string &frameId,
                      const ros::Time timestamp, const std::string &RGB);

  void
  pubNormalArrows(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &normals,
                  ros::Publisher &publisher);
};

template <typename pointType>
CloudManager<pointType>::CloudManager() = default;

template <typename pointType>
bool CloudManager<pointType>::transform(
    const typename pcl::PointCloud<pointType>::Ptr &cloudIn,
    typename pcl::PointCloud<pointType>::Ptr &cloudOut,
    const geometry_msgs::Transform &tf, const std::string &targetFrame)
{
  if (cloudIn->header.frame_id.empty())
  {
    ROS_ERROR("PCL cloud to be transformed has no frame_id. Check cloud "
              "manager transform.");
    return false;
  }

  if (cloudIn->empty())
  {
    ROS_ERROR("PCL cloud to be transformed is empty. Skip transform of the "
              "point cloud");
    return false;
  }

  pcl_ros::transformPointCloud(*cloudIn, *cloudOut, tf);
  cloudOut->header.frame_id = targetFrame;
  return true;
}

template <typename pointType>
void CloudManager<pointType>::publish(
    const typename pcl::PointCloud<pointType>::Ptr &msg,
    const ros::Publisher &publisher)
{
  if (publisher.getNumSubscribers() < 1)
    return;
  publisher.publish(*msg);
}

template <typename pointType>
void CloudManager<pointType>::passthroughFilter(
    typename pcl::PointCloud<pointType>::Ptr &cloud,
    const std::string &fieldName, double minVal, double maxVal)
{
  pcl::PassThrough<pointType> ps;
  ps.setInputCloud(cloud);
  ps.setFilterFieldName(fieldName);
  ps.setFilterLimits(minVal, maxVal);
  ps.filter(*cloud);
}

template <typename pointType>
void CloudManager<pointType>::passthroughFilter(
    typename pcl::PointCloud<pointType>::Ptr &cloud, const double minRange,
    const double maxRange, const double minZ, const double maxZ,
    const double filterAngle)
{
  if (cloud->empty())
  {
    ROS_WARN(
        "point cloud empty. Skip passthrough filtering of the point cloud");
    return;
  }

  typename pcl::PointCloud<pointType>::Ptr temp(new pcl::PointCloud<pointType>);
  temp->header = cloud->header;
  temp->points.resize(cloud->size());

  for (auto point : cloud->points)
  {
    double horizonDist = sqrt(pow(point.x, 2) + pow(point.y, 2));
    double horizonAngle = std::atan2(point.y, point.x);
    double verticalAngle = std::atan2(point.z, horizonDist);

    // Step 1: Filter out points by distance
    if (/*horizonDist*/ abs(point.x) > minRange && horizonDist < maxRange &&
        point.z < maxZ && point.z > minZ)
    {
      // Step 2: Filter out points by angle (for visualization)
      if (abs(horizonAngle) < M_PI - DEG2RAD(filterAngle / 2))
        temp->points.push_back(point);
    }
  }
  cloud.swap(temp);
}

template <typename pointType>
void CloudManager<pointType>::voxelFilter(
    typename pcl::PointCloud<pointType>::Ptr &cloud, const float lx,
    const float ly, const float lz)
{
  pcl::VoxelGrid<pointType> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(lx, ly, lz);
  vox.filter(*cloud);
}

template <typename pointType>
void CloudManager<pointType>::computeNormals(
    typename pcl::PointCloud<pointType>::Ptr &cloudIn,
    typename pcl::PointCloud<pcl::Normal>::Ptr &cloudNormalOut,
    double searchRadius)
{
  pcl::NormalEstimation<pointType, pcl::Normal> ne;
  typename pcl::search::KdTree<pointType>::Ptr kdtree;

  ne.setInputCloud(cloudIn);
  ne.setSearchMethod(kdtree);
  ne.setRadiusSearch(searchRadius);
  ne.setViewPoint(0, 0, 100);
  ne.compute(*cloudNormalOut);
}

template <typename pointType>
void CloudManager<pointType>::computeNormals(
    typename pcl::PointCloud<pointType>::Ptr &cloudIn,
    typename pcl::PointCloud<pcl::Normal>::Ptr &cloudNormalOut, float nearK)
{
  pcl::NormalEstimation<pointType, pcl::Normal> ne;
  typename pcl::search::KdTree<pointType>::Ptr kdtree;

  ne.setInputCloud(cloudIn);
  ne.setSearchMethod(kdtree);
  ne.setKSearch(nearK);
  ne.setViewPoint(0, 0, 100);
  ne.compute(*cloudNormalOut);
}

template <typename pointType>
void CloudManager<pointType>::pubNormalArrows(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &normals,
    ros::Publisher &publisher)
{
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;

  marker.header.frame_id = normals->header.frame_id;
  ros::Time timestamp;
  pcl_conversions::fromPCL(normals->header.stamp, timestamp);
  marker.header.stamp = timestamp;
  marker.ns = "Traversability";

  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  // arrow shape
  marker.scale.x = 0.2; // shaft diameter
  marker.scale.y = 0.4; // haed diameter
  marker.scale.z = 0.3; // head length
  float shaft_length = 1.3;
  // color
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  int i = 0;
  for (auto &point : normals->points)
  {
    // asd
    marker.id = i;
    ++i;

    geometry_msgs::Point start_point;
    start_point.x = point.x;
    start_point.y = point.y;
    start_point.z = point.z;
    marker.points.push_back(start_point);

    geometry_msgs::Point end_point;
    end_point.x = point.x + shaft_length * point.normal_x;
    end_point.y = point.y + shaft_length * point.normal_y;
    end_point.z = point.z + shaft_length * point.normal_z;
    marker.points.push_back(end_point);

    marker_array.markers.push_back(marker);
    marker.points.clear();
  }

  publisher.publish(marker_array);
}

template <typename pointType>
void CloudManager<pointType>::pubNormalArrow(const pcl::PointXYZINormal &normal,
                                             ros::Publisher &publisher,
                                             const std::string &frameId,
                                             const ros::Time timestamp,
                                             const std::string &rgb)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frameId;
  marker.header.stamp = timestamp;

  marker.ns = "Traversability";

  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  // arrow shape
  marker.scale.x = 0.2; // shaft diameter
  marker.scale.y = 0.4; // haed diameter
  marker.scale.z = 0.3; // head length
  float shaft_length = 1.3;
  // color
  if (rgb == "r")
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  if (rgb == "b")
  {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
  if (rgb == "g")
  {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
  }
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker.id = 0;

  geometry_msgs::Point start_point;
  start_point.x = normal.x;
  start_point.y = normal.y;
  start_point.z = normal.z;
  marker.points.push_back(start_point);

  geometry_msgs::Point end_point;
  end_point.x = normal.x + shaft_length * normal.normal_x;
  end_point.y = normal.y + shaft_length * normal.normal_y;
  end_point.z = normal.z + shaft_length * normal.normal_z;
  marker.points.push_back(end_point);

  publisher.publish(marker);
}