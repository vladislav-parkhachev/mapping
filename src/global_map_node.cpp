#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#define RESOLUTION_MAP 0.1
#define WIDTH_MAP 150
#define HEIGHT_MAP 150
#define PASSABLE_CELL 0
#define NOT_PASSABLE_CELL 100
#define UNKNOWN_CELL 50
#define K 0.5

static std::vector<int> local_map_data;
bool NewLocalMapData = false;

void callback_local_map(const nav_msgs::OccupancyGrid &data_map)
{
    local_map_data.resize(data_map.data.size());

    for (int i = 0; i < data_map.data.size(); i++)
    {
        local_map_data.at(i) = data_map.data.at(i);
    }
    NewLocalMapData = true;
}

static float odometry_data_x;
static float odometry_data_y;
static double odometry_data_teta;
bool NewOdometryData = false;

void callback_odometry(const nav_msgs::Odometry &odometry_data)
{
    odometry_data_x = odometry_data.pose.pose.position.x;
    odometry_data_y = odometry_data.pose.pose.position.y;

    tf::Quaternion q(odometry_data.pose.pose.orientation.x,
                     odometry_data.pose.pose.orientation.y,
                     odometry_data.pose.pose.orientation.z,
                     odometry_data.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odometry_data_teta = -yaw;
    NewOdometryData = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_map_node");
    ros::NodeHandle nh;
    ros::Subscriber subscriber_local_map = nh.subscribe("local_map", 50, callback_local_map);
    ros::Subscriber subscriber_odometry = nh.subscribe("/odom", 50, callback_odometry);
    ros::Publisher publisher_global_map = nh.advertise<nav_msgs::OccupancyGrid>("global_map", 50);

    tf::TransformBroadcaster odom_broadcaster;

    nav_msgs::OccupancyGrid global_map;
    global_map.header.frame_id = "global_map";
    global_map.info.resolution = RESOLUTION_MAP;
    global_map.info.width = WIDTH_MAP;
    global_map.info.height = HEIGHT_MAP;
    global_map.info.origin.position.x = -RESOLUTION_MAP * WIDTH_MAP / 2;
    global_map.info.origin.position.y = -RESOLUTION_MAP * HEIGHT_MAP / 2;
    global_map.data.resize(WIDTH_MAP * HEIGHT_MAP);

    for (unsigned int i = 0; i < WIDTH_MAP * HEIGHT_MAP; i++)
    {
       global_map.data[i] = UNKNOWN_CELL;
    }

    ros::Rate r(10);

    while (nh.ok())
    {
        if (NewOdometryData && NewLocalMapData)
        {
            geometry_msgs::Quaternion local_map_global_map_quat = tf::createQuaternionMsgFromYaw(0);
            geometry_msgs::TransformStamped local_map_global_map;
            local_map_global_map.header.stamp = ros::Time::now();
            local_map_global_map.header.frame_id = "odom";
            local_map_global_map.child_frame_id = "global_map";
            local_map_global_map.transform.translation.x = 0;
            local_map_global_map.transform.translation.y = 0;
            local_map_global_map.transform.translation.z = 0;
            local_map_global_map.transform.rotation = local_map_global_map_quat;
            odom_broadcaster.sendTransform(local_map_global_map);
            global_map.header.stamp = ros::Time::now();

            for (unsigned int j = 0; j < HEIGHT_MAP; j++)
            {
                for (unsigned int i = 0; i < WIDTH_MAP; i++)
                {
                    if (local_map_data.at(j * WIDTH_MAP + i) == NOT_PASSABLE_CELL || local_map_data.at(j * WIDTH_MAP + i) == PASSABLE_CELL)
                    {
                        double x = ((float)i - (WIDTH_MAP / 2)) * RESOLUTION_MAP + 0.24;
                        double y = ((float)j - (HEIGHT_MAP / 2)) * RESOLUTION_MAP;


                        float x_odom_final = x * cos(odometry_data_teta) + y * sin(odometry_data_teta) + odometry_data_x;
                        float y_odom_final = x * -sin(odometry_data_teta) + y * cos(odometry_data_teta) + odometry_data_y;


                        int cell_x = x_odom_final / RESOLUTION_MAP + (WIDTH_MAP / 2);
                        int cell_y = y_odom_final / RESOLUTION_MAP + (HEIGHT_MAP / 2);

                        global_map.data.at(cell_y * WIDTH_MAP + cell_x) = K * global_map.data.at(cell_y * HEIGHT_MAP + cell_x) + (1 - K) * local_map_data.at(j * WIDTH_MAP + i);
                    }
                }
            }
            publisher_global_map.publish(global_map);
            NewOdometryData = false;
            NewLocalMapData = false;
        }
        ros::spinOnce();
        r.sleep();
    }
}

