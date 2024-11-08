#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <FP_Magang/BS2PC.h>
#include <FP_Magang/PC2BS.h>
#include <FP_Magang/PC2PC.h>
#include <iostream>
#include <cmath>

#include <termios.h>
#include <unistd.h>

using namespace std;

float x = 0.0;
float y = 0.0;
float delt_x = 0.0;
float delt_y = 0.0;
float th = 0.0;
int status = 0;

bool isMoving = false;
bool reachedDest = false;
bool f_init = false;
float f_x = 0.0;
float f_y = 0.0;
float ball_x = 0.0;
float ball_y = 0.0;

ros::Publisher pub;

void normalizeVector(float* x1, float* y1) {
    float magnitude = std::sqrt((*x1) * (*x1) + (*y1) * (*y1));

    float old_x = x;
    float old_y = y;

    if (magnitude > 0.0f) {
        *x1 /= magnitude;
        *y1 /= magnitude;
    }

    delt_x = x - old_x;
    delt_y = y - old_y;
}

void updatePositionFromEncoders(float s1, float s2) {
    const float k = 0.7071;

    x = k * s1 + k * s2;
    y = -(k * s1 - k * s2);
}

char getKeyInput() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

// Calculate wheel velocities based on robot kinematics
float CalculateWheelVel(int wheel, float vx, float vy, float w) {
    float theta, speed = 100.0f;

    switch(wheel) {
        case 1:
            theta = 0.0f; break;
        case 2:
            theta = 120.0f; break;
        case 3:
            theta = 240.0f; break;
        default:
            return 0.0f;
    }

    theta = theta * M_PI / 180.0f;
    float wheelVel = cos(theta) * -(vx*speed) + sin(theta) * (vy*speed) + w;

    return wheelVel;
}
// Rotate the robot clockwise by a certain angle
void rotateRobotClockwise(float dth) {
    FP_Magang::PC2BS control_msg;

    float angular_velocity = (dth / 360.0f) * 10000.0;

    control_msg.motor1 = CalculateWheelVel(1, 0, 0, angular_velocity);
    control_msg.motor2 = CalculateWheelVel(2, 0, 0, angular_velocity);
    control_msg.motor3 = CalculateWheelVel(3, 0, 0, angular_velocity);
    // control_msg.bola_x = f_x;
    // control_msg.bola_y = f_y;
    control_msg.bola_x = ball_x;
    control_msg.bola_y = ball_y;
    pub.publish(control_msg);
}

// Global variable to track whether the robot has rotated or not
bool hasRotated = false, madeTarg = false;

float targ_x = 0.0, targ_y = 0.0, oriDist = 0.0;
bool isFirstRun = true; // New variable to track first run
float x_init = 0.0;
float y_init = 0.0;

void chaseTarget(float target_x, float target_y) {
    if (!isMoving) {
        return;
    }
    if (reachedDest) {
        return;
    }

    // Set initial position to zero on the first run
    if (isFirstRun) {
        x_init = x;
        y_init = y;
        isFirstRun = false;
    }

    // Calculate adjusted target coordinates and current coordinates
    float dx = (target_x - x_init) - (x - x_init);
    float dy = (target_y - x_init) - (y - y_init);
    float dx2 = (-target_x - x_init) - (x - x_init);

    float distance = std::sqrt(dx2 * dx2 + dy * dy);

    float stop_threshold = 20.0f;
    if (distance < stop_threshold) {
        isMoving = false;
        reachedDest = true;
        ROS_INFO("Target reached: Stopping robot.");
        
        FP_Magang::PC2BS control_msg;
        control_msg.motor1 = CalculateWheelVel(1, -(delt_x), -(delt_y), 0);
        control_msg.motor2 = CalculateWheelVel(2, -(delt_x), -(delt_y), 0);
        control_msg.motor3 = CalculateWheelVel(3, -(delt_x), -(delt_y), 0);
        control_msg.bola_x = ball_x;
        control_msg.bola_y = ball_y;
        pub.publish(control_msg);
        
        return;
    }

    if (!hasRotated) {
        float target_angle = std::atan2(dy, dx) * 180.0f / M_PI - 105.0f;

        float angle_diff = target_angle - th;
        if (angle_diff > 180) angle_diff -= 360;
        if (angle_diff < -180) angle_diff += 360;

        if (std::fabs(angle_diff) > 3.0f) {
            FP_Magang::PC2BS control_msg;
            
            float rotPer = 4.0f;
            control_msg.motor1 = CalculateWheelVel(1, 0, 0, rotPer);
            control_msg.motor2 = CalculateWheelVel(2, 0, 0, rotPer);
            control_msg.motor3 = CalculateWheelVel(3, 0, 0, rotPer);
            control_msg.bola_x = ball_x;
            control_msg.bola_y = ball_y;
            pub.publish(control_msg);
            ROS_INFO("Rotating to face target: %.2f degrees %.2f", angle_diff, target_angle);
            
            return;
        }

        hasRotated = true;
        ROS_INFO("Rotation complete. Moving towards target.");
    }

    normalizeVector(&dx, &dy);

    if (oriDist == 0.0) {
        oriDist = distance;
    }

    float vy = 0.7;
    float vx = vy * dx;
    vy *= dy;

    if (!madeTarg) {
        madeTarg = true;
        targ_x = vx;
        targ_y = vy;
    }

    FP_Magang::PC2BS control_msg;
    control_msg.motor1 = CalculateWheelVel(1, targ_x * (distance / oriDist), targ_y * (distance / oriDist), 0);
    control_msg.motor2 = CalculateWheelVel(2, targ_x * (distance / oriDist), targ_y * (distance / oriDist), 0);
    control_msg.motor3 = CalculateWheelVel(3, targ_x * (distance / oriDist), targ_y * (distance / oriDist), 0);
    control_msg.bola_x = ball_x;
    control_msg.bola_y = ball_y;
    pub.publish(control_msg);

    ROS_INFO("Moving to: X = %.2f, Y = %.2f, Distance = %.2f, Adjusted Target: (%.2f, %.2f)", target_x, target_y, distance, targ_x, targ_y);
}

int rotateCount = 0, rotateStep = 0;
void chaseRoundTarget(float target_x, float target_y) {
    if (!isMoving) {
        return;
    }
    if (reachedDest) {
        return;
    }

    float dx = (target_x - x) - (x - x);
    float dy = (target_y - y) - (y - y);
    float dx2 = (-target_x - x) - (x - x);

    float distance = std::sqrt(dx2 * dx2 + dy * dy);
    float stop_threshold = 100.0f;

    // Check if within stopping distance
    if (distance < stop_threshold) {
        if (rotateCount < 2) {
            
            float radius = 100.0f; // radius for circular path around target
            float theta_offset = 10.0f; // angle increment for smooth rotation path
            float angle_rad = (theta_offset * rotateStep) * M_PI / 180.0f; // convert to radians

            float circ_x = target_x + radius * std::cos(angle_rad);
            float circ_y = target_y + radius * std::sin(angle_rad);

            float circ_dx = circ_x - x;
            float circ_dy = circ_y - y;
            normalizeVector(&circ_dx, &circ_dy);
            float vx = 0.5f * circ_dx;
            float vy = 0.5f * circ_dy;
            normalizeVector(&vx, &vy);

            FP_Magang::PC2BS control_msg;
            control_msg.motor1 = CalculateWheelVel(1, vx, vy, 0);
            control_msg.motor2 = CalculateWheelVel(2, vx, vy, 0);
            control_msg.motor3 = CalculateWheelVel(3, vx, vy, 0);
            control_msg.bola_x = ball_x;
            control_msg.bola_y = ball_y;
            pub.publish(control_msg);

            rotateStep++;
            if (rotateStep >= 36) {
                rotateStep = 0;
                rotateCount++;
            }
            ROS_INFO("Rotating around target, rotation count: %d", rotateCount);
            return;
        } else {
            // Completed rotation, proceed to target
            isMoving = false;
            reachedDest = true;
            ROS_INFO("Target reached: Stopping robot.");

            FP_Magang::PC2BS control_msg;
            control_msg.motor1 = CalculateWheelVel(1, -(delt_x), -(delt_y), 0);
            control_msg.motor2 = CalculateWheelVel(2, -(delt_x), -(delt_y), 0);
            control_msg.motor3 = CalculateWheelVel(3, -(delt_x), -(delt_y), 0);
            control_msg.bola_x = ball_x;
            control_msg.bola_y = ball_y;
            pub.publish(control_msg);
            
            return;
        }
    }

    if (!hasRotated) {
        float target_angle = std::atan2(dy, dx) * 180.0f / M_PI - 105.0f;
        float angle_diff = target_angle - th;
        if (angle_diff > 180) angle_diff -= 360;
        if (angle_diff < -180) angle_diff += 360;

        if (std::fabs(angle_diff) > 3.0f) {
            FP_Magang::PC2BS control_msg;
            float rotPer = 4.0f;
            control_msg.motor1 = CalculateWheelVel(1, 0, 0, rotPer);
            control_msg.motor2 = CalculateWheelVel(2, 0, 0, rotPer);
            control_msg.motor3 = CalculateWheelVel(3, 0, 0, rotPer);
            control_msg.bola_x = ball_x;
            control_msg.bola_y = ball_y;
            pub.publish(control_msg);
            ROS_INFO("Rotating to face target: %.2f degrees %.2f", angle_diff, target_angle);
            return;
        }
        hasRotated = true;
        ROS_INFO("Rotation complete. Moving towards target.");
    }

    normalizeVector(&dx, &dy);

    if (oriDist == 0.0) {
        oriDist = distance;
    }

    float vy = 0.7;
    float vx = vy * dx;
    vy *= dy;

    if (!madeTarg) {
        madeTarg = true;
        targ_x = vx;
        targ_y = vy;
    }

    FP_Magang::PC2BS control_msg;
    control_msg.motor1 = CalculateWheelVel(1, targ_x * (distance / oriDist), targ_y * (distance / oriDist), 0);
    control_msg.motor2 = CalculateWheelVel(2, targ_x * (distance / oriDist), targ_y * (distance / oriDist), 0);
    control_msg.motor3 = CalculateWheelVel(3, targ_x * (distance / oriDist), targ_y * (distance / oriDist), 0);
    control_msg.bola_x = ball_x;
    control_msg.bola_y = ball_y;
    pub.publish(control_msg);

    ROS_INFO("Moving to: X = %.2f, Y = %.2f, Distance = %.2f, %.2f %.2f %.2f", target_x, target_y, distance, x, y, distance / oriDist);
}

// Callback for receiving PC2PC messages
void pc2pcCallback(const FP_Magang::PC2PC::ConstPtr& msg) {
    if (status == 2 || status == 4) {
        if (f_x != 0 && f_y != 0 || !f_init) {
            f_init = true;
            if (!isMoving && !reachedDest) {
                isMoving = true;
                hasRotated = false;
                f_x = msg->follow_x;
                f_y = msg->follow_y;
            }
        }
    }
}

void resetVars() {
    ROS_INFO("RESET STATUS");
    reachedDest = false;
    f_x = 0.0;
    f_y = 0.0;
    isMoving = false;
    f_init = false;
    hasRotated = false;
    oriDist = 0.0;
    madeTarg = false;
    rotateCount = 0;
    rotateStep = 0;
    isFirstRun = false;
}

// Callback for receiving BS2PC messages
bool isHold = false;
void bs2pcCallback(const FP_Magang::BS2PC::ConstPtr& msg) {

    if (status != msg->status && status != 0) {
        // RESET VARS
        resetVars();
    }
    status = msg->status;

    if (msg->status == 1) {
        ROS_INFO("Status is 1. Activating control system.");

        char key_input = getKeyInput();

        if (key_input == 'x') {
            ROS_INFO("Exiting control system.");
            return;
        }

        float dx = 0.0, dy = 0.0, dth = 0.0;

        switch (key_input) {
            case 'w': dy = 1.0; break;
            case 's': dy = -1.0; break;
            case 'a': dx = -1.0; break;
            case 'd': dx = 1.0; break;
            case 'q': dth = 5.0; break;
            case 'e': dth = -5.0; break;
            case 'z': isHold = true; break;
            case 'c': isHold = false; break;
            default: ROS_WARN("Invalid input.");
        }
        if (isHold) {
            ball_x = x;
            ball_y = y;    
        }

        float theta_rad = th * M_PI / 180.0f;
        float rotated_dx = cos(theta_rad) * dx - sin(theta_rad) * dy;
        float rotated_dy = sin(theta_rad) * dx + cos(theta_rad) * dy;

        FP_Magang::PC2BS control_msg;
        rotateRobotClockwise(dth);

        control_msg.motor1 = CalculateWheelVel(1, rotated_dx, rotated_dy, 0);
        control_msg.motor2 = CalculateWheelVel(2, rotated_dx, rotated_dy, 0);
        control_msg.motor3 = CalculateWheelVel(3, rotated_dx, rotated_dy, 0);
        control_msg.bola_x = ball_x;
        control_msg.bola_y = ball_y;

        pub.publish(control_msg);

    } else if (msg->status == 2) {

        ball_x = f_x;
        ball_y = f_y;

        FP_Magang::PC2BS control_msg;

        if (!reachedDest) {
            ROS_INFO("Status is 2");

            if (f_x != 0 && f_y != 0) {
                chaseTarget(-f_x, f_y);
            }
        } else {

            control_msg.motor1 = CalculateWheelVel(1, 0, 0, 0);
            control_msg.motor2 = CalculateWheelVel(2, 0, 0, 0);
            control_msg.motor3 = CalculateWheelVel(3, 0, 0, 0);
            ball_x = x;
            ball_y = y;
        }

        control_msg.bola_x = ball_x;
        control_msg.bola_y = ball_y;
        pub.publish(control_msg);

    } else if (msg->status == 3) {

        if (f_x != msg->tujuan_x && f_y != msg->tujuan_y) {
            resetVars();
            f_x = msg->tujuan_x;
            f_y = msg->tujuan_y;
        }

        ball_x = f_x;
        ball_y = f_y;
    
        FP_Magang::PC2BS control_msg;
        
        if (!reachedDest) {
            ROS_INFO("Status is 3 %.2f %.2f", f_x, f_y);

            isMoving = true;
            chaseTarget(-f_x, f_y);
        } else {

            control_msg.motor1 = CalculateWheelVel(1, 0, 0, 0);
            control_msg.motor2 = CalculateWheelVel(2, 0, 0, 0);
            control_msg.motor3 = CalculateWheelVel(3, 0, 0, 0);
            ball_x = x;
            ball_y = y;
        }

        control_msg.bola_x = ball_x;
        control_msg.bola_y = ball_y;
        pub.publish(control_msg);
    } else if (msg->status == 4) {

        ball_x = f_x;
        ball_y = f_y;

        FP_Magang::PC2BS control_msg;

        if (!reachedDest) {
            ROS_INFO("Status is 4");

            if (f_x != 0 && f_y != 0) {
                chaseRoundTarget(-f_x, f_y);
            }
        } else {

            control_msg.motor1 = CalculateWheelVel(1, 0, 0, 0);
            control_msg.motor2 = CalculateWheelVel(2, 0, 0, 0);
            control_msg.motor3 = CalculateWheelVel(3, 0, 0, 0);
            ball_x = x;
            ball_y = y;
        }

        control_msg.bola_x = ball_x;
        control_msg.bola_y = ball_y;
        pub.publish(control_msg);

    }

    // NORMALIZE THETA
    th = std::fmod(msg->th, 360.0f);
    if (th < -180.0f) th += 360.0f;
    if (th > 180.0f) th -= 360.0f;

    updatePositionFromEncoders(msg->enc_left, msg->enc_right);

    // FP_Magang::PC2BS control_msg;
    // pub.publish(control_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    pub = nh.advertise<FP_Magang::PC2BS>("/pc2bs", 1);
    ros::Subscriber sub = nh.subscribe("/bs2pc", 1, bs2pcCallback);
    ros::Subscriber sub_pc = nh.subscribe("/pc2pc", 1, pc2pcCallback);

    ros::spin();
    return 0;
}