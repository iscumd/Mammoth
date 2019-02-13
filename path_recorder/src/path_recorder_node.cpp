//
// Created by isc on 1/10/19.
//

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>

void write_to_file(std::ofstream &out, double x, double y, double theta) {
    out << x << " " << y << " " << theta << std::endl;
}

double
distance(const double &x1, const double &y1, const double &x2,
         const double &y2)  // function overloading so usable with point2d
{
    return std::sqrt(std::pow((x2 - x1), 2) + std::pow((y2 - y1), 2));
}

int main(int argc, char **argv) {
    std::string file = "";
    if (argc == 2) {
        file = argv[1];
    } else {
        std::cout << "Usage: path_recorder_node OutputFile" << std::endl;
        return 1;
    }
    ros::init(argc, argv, "path_recorder");

    std::ofstream out;
    out.open(file);
    ros::NodeHandle node;
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    double x_prev = 0, y_prev = 0, theta_prev = 0;
    while (node.ok()) {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/map", "/base_link",
                                     ros::Time(0), transform);

        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        double x_pos = transform.getOrigin().x();
        double y_pos = transform.getOrigin().y();
        tf::Matrix3x3 mat(transform.getRotation());
        double r, p, y;
        mat.getRPY(r, p, y);
        double theta = y;
//        std::cout << x_pos << " " << y_pos << " " << theta << std::endl;
        if (distance(x_prev, y_prev, x_pos, y_pos) > 0.3) {
            write_to_file(out, x_pos, y_pos, theta);
            x_prev = x_pos;
            y_prev = y_pos;
            std::cout << "writing " << x_pos << " " << y_pos << " " << theta << "to " << file << std::endl;
        }
        rate.sleep();
    }
    out.close();
    return 0;
};