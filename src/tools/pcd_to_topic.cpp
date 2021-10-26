#include <string>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cout << "usage: pcd_to_topic [pcd_path] [topic_name]" << std::endl;
        return EXIT_FAILURE;
    }
    ros::init(argc, argv, std::string(argv[2]) + "_publisher");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(argv[2], 1);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    sensor_msgs::PointCloud2 output;
    pcl::io::loadPCDFile(argv[1], cloud); //Modify the path of your pcd file
    //Convert the cloud to ROS message
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map"; //this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
                                     //! ! ! This step needs attention, it is the fixed_frame of rviz behind!!! Knock the blackboard and draw the key points.
    // ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        sleep(1);
    }
    return 0;
}