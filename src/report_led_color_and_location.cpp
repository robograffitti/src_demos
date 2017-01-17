#include <ros/console.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <srcsim/Console.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define INTERVAL 0.25
#define OFFSET 0.25
#define COLOR_THRESHOLD 225

using namespace message_filters;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace srcsim;
using namespace std;

int i = 0;
ros::Publisher light_pub;
ros::Time time_prev;

void callback(const PointCloud2ConstPtr& cloud_in, const PointStampedConstPtr& pts_in)
{
  // 1. Obtain PointStamped
  PointStamped pts = *pts_in;

  // 2. Process PointCloud2 and Obtain RGB
  // PointCloud2 cloud  = *cloud_in;
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(*cloud_in, *cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(*cloud, *cloud_xyzrgb);

  // TODO: exclude small noise values in RGB

  // 3. Store the obtained values in srcsim/Console message.
  Console light;
  light.x = pts.point.x;
  light.y = pts.point.y;
  light.z = pts.point.z;
  light.r = cloud_xyzrgb->points[0].r;
  light.g = cloud_xyzrgb->points[0].g;
  light.b = cloud_xyzrgb->points[0].b;

  if (light.r > COLOR_THRESHOLD)
    light.r = 1;
  else
    light.r = 0;

  if (light.g > COLOR_THRESHOLD)
    light.g = 1;
  else
    light.g = 0;

  if (light.b > COLOR_THRESHOLD)
    light.b = 1;
  else
    light.b = 0;

  // 4. Publication Control Sequencen
  if ( i == 0 | (pts.header.stamp - time_prev).toSec() > INTERVAL ) {
    light_pub.publish(light);
    i++;
    // for debugging
    cout << "Light No." << i << endl;
    cout << "x = " << light.x << endl;
    cout << "y = " << light.y << endl;
    cout << "z = " << light.z << endl;
    cout << "R = " << light.r << endl;
    cout << "G = " << light.g << endl;
    cout << "B = " << light.b << endl;
    cout << endl;
  }

  // 5. Store the time stamp from the previous sequence
  time_prev = pts.header.stamp;

  // Redundant old codes...
  /* if ( i == 0) {
    // initial publication
    light_pub.publish(light);
    i++;
    cout << "Light No." << i << endl;
  } else {
    if ( (pts.header.stamp - time_prev).toSec() > INTERVAL ) {
      // publication based on time stamp control after initial publication
      light_pub.publish(light);
      i++;
    }
    cout << "Light No." << i << endl;
    } */
  // ROS_INFO( "point time stamp: %d", pts.header.stamp.toSec() );
  // ROS_INFO( "cloud time stamp: %d", cloud_in->header.stamp.toSec() );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "report_led_color_and_location_node");

  ros::NodeHandle nh;
  Subscriber<PointCloud2> cloud_sub(nh, "/RGB_color_filter/rgb_output", 1);
  Subscriber<PointStamped> point_sub(nh, "/RGB_color_filter/centroid_publisher/output/point", 1);
  light_pub = nh.advertise<Console>("/srcsim/qual1/light", 1);

  typedef sync_policies::ApproximateTime<PointCloud2, PointStamped> SyncPolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
  Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud_sub, point_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
