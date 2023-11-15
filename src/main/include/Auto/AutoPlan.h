#pragma once

#ifndef MY_PATH_PLAN_H
#define MY_PATH_PLAN_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include "commonauto/AutoStep.h"
#include "swerve/src/include/SwerveTrain.h"

class AutoPlan : public AutoStep {
public:
    AutoPlan() : AutoStep("AutoPlan") {}

    void init() {
        readPathData("Auto_Plan.csv");
    }

    void execute() {
        // Loop through each point on the path
        for (const auto& point : path) {
            // Calculate the desired movement to reach the current point
            double x_error = point.first - robot_x;
            double y_error = point.second - robot_y;
            double desired_heading = atan2(y_error, x_error);
            double heading_error = desired_heading - robot_angle;

            // Use the errors to control the robot's movement
            double x_speed = x_error * kP_x;
            double y_speed = y_error * kP_y;
            double heading_speed = heading_error * kP_heading;
            SwerveTrain::GetInstance().Drive(x_speed, y_speed, heading_speed, false, false, 1);

            // Update the robot's position and angle
            robot_x += x_speed;
            robot_y += y_speed;
            robot_angle += heading_speed;
        }
    }

private:
    const std::vector<std::pair<double, double>> path; // Stores the path data as pairs of x and y coordinates
    double robot_x = 0; // The robot's current x position
    double robot_y = 0; // The robot's current y position
    double robot_angle = 0; // The robot's current angle (in radians)

    const double kP_x = 0.1; // Proportional constant for x movement
    const double kP_y = 0.1; // Proportional constant for y movement
    const double kP_heading = 0.1; // Proportional constant for heading

    void readPathData(const std::string& file_name) {
        std::ifstream file(file_name);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open path data file: " + file_name);
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string x_str, y_str;
            if (!(iss >> x_str >> y_str)) {
                throw std::runtime_error("Error reading path data from file: " + file_name);
            }

            double x = std::stod(x_str);
        }
    }
};

#endif
