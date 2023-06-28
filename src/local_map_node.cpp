#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define RESOLUTION_MAP 0.1
#define WIDTH_MAP 150
#define HEIGHT_MAP 150
#define PASSABLE_CELL 0
#define NOT_PASSABLE_CELL 100
#define UNKNOWN_CELL 50

static std::vector<float> ranges;
static unsigned long ranges_size;
static float range_min;
static float range_max;
static float angle_min;
static float angle_increment;
bool NewLaserScanData = false;

void callback_laser_scan(const sensor_msgs::LaserScan &laser_scan_data)
{
    ranges = laser_scan_data.ranges;
    ranges_size = laser_scan_data.ranges.size();
    range_min = laser_scan_data.range_min;
    range_max = laser_scan_data.range_max;
    angle_min = laser_scan_data.angle_min;
    angle_increment = laser_scan_data.angle_increment;
    NewLaserScanData = true;
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
    odometry_data_teta = yaw;
    NewOdometryData = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle nh;

    ros::Subscriber subscriber_laser_scan = nh.subscribe("/scan", 50, callback_laser_scan);
    ros::Subscriber subscriber_odometry = nh.subscribe("/odom", 50, callback_odometry);

    ros::Publisher publisher_local_map = nh.advertise<nav_msgs::OccupancyGrid>("local_map", 50);
    nav_msgs::OccupancyGrid local_map;
    local_map.header.frame_id = "local_map";
    local_map.info.resolution = RESOLUTION_MAP;
    local_map.info.width = WIDTH_MAP;
    local_map.info.height = HEIGHT_MAP;
    local_map.info.origin.position.x = -RESOLUTION_MAP * WIDTH_MAP / 2;
    local_map.info.origin.position.y = -RESOLUTION_MAP * HEIGHT_MAP / 2;
    local_map.data.resize(WIDTH_MAP * HEIGHT_MAP);

    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate r(10);

    while (nh.ok())
    {
        if (NewLaserScanData && NewOdometryData)
        {
            geometry_msgs::Quaternion base_link_odom_quat = tf::createQuaternionMsgFromYaw(odometry_data_teta);
            geometry_msgs::TransformStamped base_link_odom;
            base_link_odom.header.stamp = ros::Time::now();
            base_link_odom.header.frame_id = "odom";
            base_link_odom.child_frame_id = "base_link";
            base_link_odom.transform.translation.x = odometry_data_x;
            base_link_odom.transform.translation.y = odometry_data_y;
            base_link_odom.transform.translation.z = 0;
            base_link_odom.transform.rotation = base_link_odom_quat;
            odom_broadcaster.sendTransform(base_link_odom);

            geometry_msgs::Quaternion laser_base_link_quat = tf::createQuaternionMsgFromYaw(0);
            geometry_msgs::TransformStamped laser_base_link;
            laser_base_link.header.stamp = ros::Time::now();
            laser_base_link.header.frame_id = "base_link";
            laser_base_link.child_frame_id = "laser";
            laser_base_link.transform.translation.x = 0.24;
            laser_base_link.transform.translation.y = 0.0;
            laser_base_link.transform.translation.z = 0.0;
            laser_base_link.transform.rotation = laser_base_link_quat;
            odom_broadcaster.sendTransform(laser_base_link);

            geometry_msgs::Quaternion odom_local_map_quat = tf::createQuaternionMsgFromYaw(0);
            geometry_msgs::TransformStamped odom_local_map;
            odom_local_map.header.stamp = ros::Time::now();
            odom_local_map.header.frame_id = "laser";
            odom_local_map.child_frame_id = "local_map";
            odom_local_map.transform.translation.x = 0;
            odom_local_map.transform.translation.y = 0;
            odom_local_map.transform.translation.z = 0;
            odom_local_map.transform.rotation = odom_local_map_quat;
            odom_broadcaster.sendTransform(odom_local_map);
            local_map.header.stamp = ros::Time::now();

            for (unsigned int i = 0; i < WIDTH_MAP * HEIGHT_MAP; i++)
            {
               local_map.data[i] = UNKNOWN_CELL;
            }

            for (unsigned int range_i = 0; range_i <= ranges_size; range_i++) // Проходимся по всем элементам массива range из callback функции
            {
                float current_range = ranges[range_i]; // Инициалиизруем текущее значение длины и присваем ей значение i-го илемента из пришедшего массива ranges
                if (current_range > range_min && current_range < range_max) // Проверяем, входит ли значение в диапазон от 0 до 6 метров (чтобы исключить inf)
                {
                    float current_angle = angle_min + angle_increment * range_i;      // Считаем текущий угол
                    int x = current_range * sin(current_angle) / RESOLUTION_MAP + WIDTH_MAP / 2;  // Считаем координату по х отразивщегося луча (получаем координату в ячайках)
                    int y = current_range * cos(current_angle) / RESOLUTION_MAP + WIDTH_MAP / 2;  // Считаем координату по у отразивщегося луча (получаем координату в ячайках)
                    local_map.data[y + x * WIDTH_MAP] = NOT_PASSABLE_CELL;                     // Заполняем получившуюся ячейку значением 100 (т.е в данной ячейке находится препятствие)

                    // Выполняем трассировку каждого луча для получения проходимых областей
                    // начинаем с текущей координаты ячейки - 0.1 и уменьшаем расстояние с каждым
                    // шагом на 0.1, вычисляя при этом координаты ячеек и заполняя их 0 (т.е они проходимы)
                    for (current_range -= RESOLUTION_MAP; current_range > 0; current_range -= RESOLUTION_MAP)
                    {
                        x = current_range * sin(current_angle) / RESOLUTION_MAP + WIDTH_MAP / 2; // Считаем координату х ячейки,в которой нет препятствия
                        y = current_range * cos(current_angle) / RESOLUTION_MAP + WIDTH_MAP / 2; // Считаем координату у ячейки,в которой нет препятствия
                        local_map.data[y + x * WIDTH_MAP] = PASSABLE_CELL;                        // Заполняем получившуюся ячейку значением 0 (т.е в данной ячейке находится препятствие)
                    }
                }
            }    
            publisher_local_map.publish(local_map);
            NewLaserScanData = false;
            NewOdometryData = false;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}


