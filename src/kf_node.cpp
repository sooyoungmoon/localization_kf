#include <math.h>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/spawn.hpp"

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

class MyKFNode : public rclcpp::Node
{
public:
  MyKFNode()
  : Node("my_kf_node")
  {   
    subscription_twist = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", 10, std::bind(&MyKFNode::handle_twist, this, std::placeholders::_1));

    subscription_pose = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&MyKFNode::handle_pose, this, std::placeholders::_1));

    subscription_pose_turtle2 = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle2/pose", 10, std::bind(&MyKFNode::handle_pose_turtle2, this, std::placeholders::_1));

    mean[0] = x0;
    mean[1] = y0;
    mean[2] = theta0;
    mean[3] = vx0;
    mean[4] = vy0;
    mean[5] = vtheta0;

    state[0] = mean[0];
    state[1] = mean[1];
    state[2] = mean[2];
    state[3] = mean[3];
    state[4] = mean[4];
    state[5] = mean[5];

    client = this->create_client<turtlesim::srv::Spawn>("spawn");
    publisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
    
    timer = this->create_wall_timer(
      std::chrono::milliseconds(Time_Interval_Millisec), std::bind(&MyKFNode::timer_callback, this));
  }

private:

    void timer_callback()
    {
        //RCLCPP_INFO(this->get_logger(), "Timer callback");
        rclcpp::Time current_time = this->get_clock()->now();
        

        if (!client->service_is_ready()) {
            //RCLCPP_INFO(this->get_logger(), "service not available");
            return;
        }

        if (!spawned) {
          auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
          request->x = x0;
          request->y = y0;
          request->theta = theta0;
          request->name = "turtle2";
          auto result = client->async_send_request(request);
          
          spawned = true;
        }

         if ( (current_time.seconds() - last_time_twist) > 1.0 ) // because turtlesim moves for 1 second for each twist message
        {
            control[V] = 0.0; // no control input at now
            control[W] = 0.0;            
        }
        
        //RCLCPP_INFO(this->get_logger(), "Current time: %f", current_time.seconds());
        //RCLCPP_INFO(this->get_logger(), "Last time twist: %f", last_time_twist);
        //RCLCPP_INFO(this->get_logger(), "Last time timer: %f", last_time_timer);

        if (last_time_twist < last_time_timer) {
            delta_t_control = current_time.seconds() - last_time_timer; // not the first state prediction since the control input
        }
        else
        {
            delta_t_control = current_time.seconds() - last_time_twist; // the first state prediction since the control input
            //RCLCPP_INFO(this->get_logger(), "Current time: %f", current_time.seconds());
            //RCLCPP_INFO(this->get_logger(), "Last time twist: %f", last_time_twist);
            //RCLCPP_INFO(this->get_logger(), "Delta t control: %f", delta_t_control);
        }        
        //RCLCPP_INFO(this->get_logger(), "Delta t control: %f", delta_t_control);

        RCLCPP_INFO(this->get_logger(), "(Ground truth) last_pose: x=%f, y=%f, theta=%f", last_pose.x, last_pose.y, last_pose.theta);

        RCLCPP_INFO(this->get_logger(), "Delta t control: %f", delta_t_control);
        if ( fabs(control[V]) > 0.0 || fabs(control[W]) > 0.0) {
        //if ( control[V] > 0.0 || control[W] > 0.0) {
            kalman_filter_prediction(mean, covariance, control);
            RCLCPP_INFO(this->get_logger(), "(Prediction) State: x=%f, y=%f, theta=%f, vx = %f, vy = %f, vtheta = %f", state[X], state[Y], state[THETA], state[VX], state[VY], state[VTHETA]);  
        }        

        RCLCPP_INFO(this->get_logger(), "Covariance:");
        for (int i = 0; i < 6; ++i) {
            std::stringstream ss;
            for (int j = 0; j < 6; ++j) {
                ss << covariance[i][j] << " ";
            }
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        }

        //RCLCPP_INFO(this->get_logger(), "Delta Pose: x=%f, y=%f, theta=%f", delta_pose[DELTA_X], delta_pose[DELTA_Y], delta_pose[DELTA_THETA]);
        
        bool update = false;

        if ( fabs(delta_pose[DELTA_X]) > 0.1 || fabs(delta_pose[DELTA_Y]) > 0.1 || fabs(delta_pose[DELTA_THETA]) > 0.1 ) {        
            //RCLCPP_INFO(this->get_logger(), "update() is to be called");
            kalman_filter_update(mean, covariance, delta_pose);       
            update = true;
        }

        state[0] = mean[0];
        state[1] = mean[1];
        state[2] = mean[2];
        state[3] = mean[3];
        state[4] = mean[4];
        state[5] = mean[5];

        if (update == true) {
            RCLCPP_INFO(this->get_logger(), "(Update) State: x=%f, y=%f, theta=%f", state[X], state[Y], state[THETA]);   
        }
        delta_pose[DELTA_X] = 0.0; // reset the delta x, y, theta
        delta_pose[DELTA_Y] = 0.0;
        delta_pose[DELTA_THETA] = 0.0;

        last_time_timer = this->get_clock()->now().seconds();
    }

    void kalman_filter_prediction(const double old_mean[], const double (*old_cov)[6], const double control_input[]) {
        RCLCPP_INFO(this->get_logger(), "Kalman Filter Prediction");
        //RCLCPP_INFO(this->get_logger(), "Kalman Filter Prediction: v=%f, w=%f", control_input[V], control_input[W]);
        //RCLCPP_INFO(this->get_logger(), "delta_t_control: %f", delta_t_control);

        RCLCPP_INFO(this->get_logger(), "Linear velocity: %f", control[V]);
        RCLCPP_INFO(this->get_logger(), "Angular velocity: %f", control[W]);

        Eigen::Matrix<double, 6, 6> A;

        A << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        /*
        A << 1, 0, 0, delta_t_control, 0, 0,
            0, 1, 0, 0, delta_t_control, 0,
            0, 0, 1, 0, 0, delta_t_control,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
        */
        
        Eigen::Matrix<double, 6, 2> B;     

        B << delta_t_control * cos(state[THETA]), 0,
             delta_t_control * sin(state[THETA]), 0,
            0, delta_t_control,
            cos(state[THETA]), 0,
            sin(state[THETA]), 0,
            0, 1;

        /*
        B << 0.5 * delta_t_control * delta_t_control * cos(state[THETA]), 0,
            0.5 * delta_t_control * delta_t_control * sin(state[THETA]), 0,
            0, delta_t_control,
            cos(state[THETA]), 0,
            sin(state[THETA]), 0,
            0, 1;
        */
        Eigen::Matrix<double, 6, 6> StateTransitionNoise;
        StateTransitionNoise << 0.1, 0, 0, 0, 0, 0,
                             0, 0.1, 0, 0, 0, 0,
                             0, 0, 0.1, 0, 0, 0,
                                0, 0, 0, 0.1, 0, 0,
                                0, 0, 0, 0, 0.1, 0,
                                0, 0, 0, 0, 0, 0.1;
             
        Eigen::Matrix<double, 6, 1> Mean;
        Mean << old_mean[0],
             old_mean[1],
             old_mean[2],
             old_mean[3],
             old_mean[4],
             old_mean[5];

        Eigen::Matrix<double, 2, 1> U;
        U << control_input[V],
             control_input[W];

        Eigen::Matrix<double, 6, 6> Cov;
        Cov << old_cov[0][0], old_cov[0][1], old_cov[0][2], old_cov[0][3], old_cov[0][4], old_cov[0][5],
               old_cov[1][0], old_cov[1][1], old_cov[1][2], old_cov[1][3], old_cov[1][4], old_cov[1][5],
               old_cov[2][0], old_cov[2][1], old_cov[2][2], old_cov[2][3], old_cov[2][4], old_cov[2][5],
                old_cov[3][0], old_cov[3][1], old_cov[3][2], old_cov[3][3], old_cov[3][4], old_cov[3][5],
                old_cov[4][0], old_cov[4][1], old_cov[4][2], old_cov[4][3], old_cov[4][4], old_cov[4][5],
                old_cov[5][0], old_cov[5][1], old_cov[5][2] , old_cov[5][3], old_cov[5][4], old_cov[5][5];

        Eigen::Matrix<double, 6, 1> New_Mean = A * Mean + B * U;
       
        if (New_Mean(2, 0) > M_PI) {
            New_Mean(2, 0) -= 2 * M_PI;
        } else if (New_Mean(2, 0) < -M_PI) {
            New_Mean(2, 0) += 2 * M_PI;
        }
        if(New_Mean(5, 0) > M_PI) {
            New_Mean(5, 0) -= 2 * M_PI;
        } else if (New_Mean(5, 0) < -M_PI) {
            New_Mean(5, 0) += 2 * M_PI;
        }

        RCLCPP_INFO(this->get_logger(), "Old Mean: x=%f, y=%f, theta=%f, vx=%f, vy=%f, vtheta=%f", old_mean[0], old_mean[1], old_mean[2], old_mean[3], old_mean[4], old_mean[5]);
        RCLCPP_INFO(this->get_logger(), "New Mean: x=%f, y=%f, theta=%f, vx=%f, vy=%f, vtheta=%f", New_Mean(0, 0), New_Mean(1, 0), New_Mean(2, 0), New_Mean(3, 0), New_Mean(4, 0), New_Mean(5, 0));
        
        Eigen::Matrix<double, 6, 6> New_Cov = A * Cov * A.transpose() + StateTransitionNoise;
    
        mean[0] = New_Mean(0, 0);
        mean[1] = New_Mean(1, 0);
        mean[2] = New_Mean(2, 0);
        mean[3] = New_Mean(3, 0);
        mean[4] = New_Mean(4, 0);
        mean[5] = New_Mean(5, 0);

        covariance[0][0] = New_Cov(0, 0);
        covariance[0][1] = New_Cov(0, 1);
        covariance[0][2] = New_Cov(0, 2);
        covariance[1][0] = New_Cov(1, 0);
        covariance[1][1] = New_Cov(1, 1);
        covariance[1][2] = New_Cov(1, 2);
        covariance[2][0] = New_Cov(2, 0);
        covariance[2][1] = New_Cov(2, 1);
        covariance[2][2] = New_Cov(2, 2);        
    }

    void kalman_filter_update(const double old_mean[], const double (*old_cov)[6], const double delta_pose[]) {
        
        //RCLCPP_INFO(this->get_logger(), "\n");
        //RCLCPP_INFO(this->get_logger(), "Kalman Filter Update");
        //RCLCPP_INFO(this->get_logger(), "Kalman Filter Update: delta_x=%f, delta_y=%f, delta_theta=%f", delta_pose[DELTA_X], delta_pose[DELTA_Y], delta_pose[DELTA_THETA]);
        //RCLCPP_INFO(this->get_logger(), "Covariance Matrix:");
        for (int i = 0; i < 6; ++i) {
            std::stringstream ss;
            for (int j = 0; j < 6; ++j) {
                ss << old_cov[i][j] << " ";
            }
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        }
        Eigen::Matrix<double, 3, 6> C;
        C << 0, 0, 0, Time_Interval_Millisec, 0, 0,
                       0, 0, 0, 0, Time_Interval_Millisec, 0,
                       0, 0, 0, 0, 0, Time_Interval_Millisec;  

        Eigen::Matrix<double, 6, 1> Mean;
        Mean << old_mean[0],
             old_mean[1],
             old_mean[2],
             old_mean[3],
             old_mean[4],
             old_mean[5]; 

        Eigen::Matrix<double, 6, 6> Cov;
        Cov << old_cov[0][0], old_cov[0][1], old_cov[0][2], old_cov[0][3], old_cov[0][4], old_cov[0][5],
               old_cov[1][0], old_cov[1][1], old_cov[1][2], old_cov[1][3], old_cov[1][4], old_cov[1][5],
               old_cov[2][0], old_cov[2][1], old_cov[2][2], old_cov[2][3], old_cov[2][4], old_cov[2][5],
                old_cov[3][0], old_cov[3][1], old_cov[3][2], old_cov[3][3], old_cov[3][4], old_cov[3][5],
                old_cov[4][0], old_cov[4][1], old_cov[4][2], old_cov[4][3], old_cov[4][4], old_cov[4][5],
                old_cov[5][0], old_cov[5][1], old_cov[5][2] , old_cov[5][3], old_cov[5][4], old_cov[5][5];


        RCLCPP_INFO(this->get_logger(), "Covariance Matrix:");
        for (int i = 0; i < 6; ++i) {
            std::stringstream ss;
            for (int j = 0; j < 6; ++j) {
                ss << Cov(i, j) << " ";
            }
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        }

        Eigen::Matrix<double, 3, 3> MeasurementNoise; // covariance for the measurement noise
        
        MeasurementNoise << 0.1, 0, 0,
                            0, 0.1, 0,
                            0, 0, 0.1;

        Eigen::Matrix<double, 3, 1> Z; // measurement
        Z << delta_pose[DELTA_X],
                delta_pose[DELTA_Y],
                delta_pose[DELTA_THETA];

        Eigen::Matrix<double, 3, 3> Covariance = C * Cov * C.transpose() + MeasurementNoise;

        //RCLCPP_INFO(this->get_logger(), "Covariance:");
        for (int i = 0; i < 3; ++i) {
            std::stringstream ss;
            for (int j = 0; j < 3; ++j) {
                ss << Covariance(i, j) << " ";
            }
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        }
        Eigen::Matrix<double, 6, 3> KalmanGain = Cov * C.transpose() * Covariance.inverse();

        RCLCPP_INFO(this->get_logger(), "Kalman Gain:");
        for (int i = 0; i < 6; ++i) {
            std::stringstream ss;
            for (int j = 0; j < 3; ++j) {
            ss << KalmanGain(i, j) << " ";
            }
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Measurement: x=%f, y=%f, theta=%f", Z(0, 0), Z(1, 0), Z(2, 0));
        Eigen::Matrix<double, 3, 1> expected_measurement = C * Mean;
        RCLCPP_INFO(this->get_logger(), "Expected Measurement: x=%f, y=%f, theta=%f", expected_measurement(0, 0), expected_measurement(1, 0), expected_measurement(2, 0));

        Eigen::Matrix<double, 6, 1> gain = KalmanGain * (Z - expected_measurement);

        //printf("Gain: %f %f %f %f %f %f\n", gain(0, 0), gain(1, 0), gain(2, 0), gain(3, 0), gain(4, 0), gain(5, 0));

        Eigen::Matrix<double, 6, 1> New_Mean = Mean + gain;
        if (New_Mean(2, 0) > M_PI) {
            New_Mean(2, 0) -= 2 * M_PI;
        } else if (New_Mean(2, 0) < -M_PI) {
            New_Mean(2, 0) += 2 * M_PI;
        }
        if(New_Mean(5, 0) > M_PI) {
            New_Mean(5, 0) -= 2 * M_PI;
        } else if (New_Mean(5, 0) < -M_PI) {
            New_Mean(5, 0) += 2 * M_PI;
        }

        //RCLCPP_INFO(this->get_logger(), "Updated Mean: x=%f, y=%f, theta=%f, vx = %f, vy = %f, vtheta = %f ", New_Mean(0, 0), New_Mean(1, 0), New_Mean(2, 0), New_Mean(3, 0), New_Mean(4, 0), New_Mean(5, 0));

        Eigen::Matrix<double, 6, 6> New_Cov = (Eigen::Matrix<double, 6, 6>::Identity() - KalmanGain * C) * Cov;

        mean[0] = New_Mean(0, 0);
        mean[1] = New_Mean(1, 0);
        mean[2] = New_Mean(2, 0);
        mean[3] = New_Mean(3, 0);
        mean[4] = New_Mean(4, 0);
        mean[5] = New_Mean(5, 0);

        covariance[0][0] = New_Cov(0, 0);
        covariance[0][1] = New_Cov(0, 1);
        covariance[0][2] = New_Cov(0, 2);
        covariance[1][0] = New_Cov(1, 0);
        covariance[1][1] = New_Cov(1, 1);
        covariance[1][2] = New_Cov(1, 2);
        covariance[2][0] = New_Cov(2, 0);
        covariance[2][1] = New_Cov(2, 1);
        covariance[2][2] = New_Cov(2, 2);

        RCLCPP_INFO(this->get_logger(), "Updated Covariance:");
        for (int i = 0; i < 6; ++i) {
            std::stringstream ss;
            for (int j = 0; j < 6; ++j) {
                ss << New_Cov(i, j) << " ";
            }
            //RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        }
       
    }



    void handle_twist(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
      //RCLCPP_INFO(this->get_logger(), "Received twist message");
      rclcpp::Time current_time = this->get_clock()->now();
      
      control[V] = msg->linear.x;
      control[W] = msg->angular.z;
      // print the linear velocity in the twist msg
      //RCLCPP_INFO(this->get_logger(), "linear velocity: x=%f, y=%f, z=%f", msg->linear.x, msg->linear.y, msg->linear.z);
      // print the angular velocity in the twist msg
      //RCLCPP_INFO(this->get_logger(), "angular velocity: x=%f, y=%f, z=%f", msg->angular.x, msg->angular.y, msg->angular.z);
    
      last_time_twist = current_time.seconds();
      last_twist = *msg;
      //kalman_filter_prediction(mean, covariance, control);
    }

    void handle_pose_turtle2(const std::shared_ptr<turtlesim::msg::Pose> msg)
    {
      //RCLCPP_INFO(this->get_logger(), "Received pose message turtle2");
      state_turtle2[X] = msg->x;
      state_turtle2[Y] = msg->y;
      state_turtle2[THETA] = msg->theta;

      if (spawned) {
            geometry_msgs::msg::Twist msg;
            
            double translation_x = state[X] - state_turtle2[X];
            double translation_y = state[Y] - state_turtle2[Y];
            //RCLCPP_INFO(this->get_logger(), "Translation: x=%f, y=%f", translation_x, translation_y);
            
            double distance = sqrt(pow(translation_x, 2) + pow(translation_y, 2));
            //RCLCPP_INFO(this->get_logger(), "Distance: %f", distance);
            if (distance < 0.1) {
                msg.linear.x = 0.0;
                double angle = state[THETA];
                double angle_difference = angle - state_turtle2[THETA]; // be careful to use the turtle2's theta, not the turtle1's theta
                if (angle_difference > M_PI) {
                    angle_difference -= 2 * M_PI;
                } else if (angle_difference < -M_PI) {
                    angle_difference += 2 * M_PI;
                }
                msg.angular.z = angle_difference;
            }
            else {
                msg.linear.x = 0.5 * distance;

                double angle = atan2(translation_y, translation_x);
                double angle_difference = angle - state_turtle2[THETA]; // be careful to use the turtle2's theta, not the turtle1's theta
                if (angle_difference > M_PI) {
                    angle_difference -= 2 * M_PI;
                } else if (angle_difference < -M_PI) {
                    angle_difference += 2 * M_PI;
                }
                //msg.angular.z = angle;
                msg.angular.z = angle_difference;
            }
            publisher->publish(msg);
        }

    }

    void handle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
    {
        //RCLCPP_INFO(this->get_logger(), "Received pose message");
        rclcpp::Time current_time = this->get_clock()->now();
        //RCLCPP_INFO(this->get_logger(), "Current time: %f", current_time.seconds());
        //RCLCPP_INFO(this->get_logger(), "last_time_pose: %f, last_time_twist: %f", last_time_pose, last_time_twist);
        // print the position in the pose msg
        //RCLCPP_INFO(this->get_logger(), "position: x=%f, y=%f", msg->x, msg->y);
        // print the orientation in the pose msg
        //RCLCPP_INFO(this->get_logger(), "orientation: theta=%f", msg->theta);
        // prepare transform message
        

        delta_t_pose = current_time.seconds() - last_time_timer;

        delta_pose[DELTA_X] += (msg->x - last_pose.x); // accumulate the delta x, y, theta untilel the next timer callback
        delta_pose[DELTA_Y] += (msg->y - last_pose.y);
        delta_pose[DELTA_THETA] += (msg->theta - last_pose.theta);

        if (delta_pose[DELTA_THETA] > M_PI) {
                delta_pose[DELTA_THETA] -= 2 * M_PI;
            } else if (delta_pose[DELTA_THETA] < -M_PI) {
                delta_pose[DELTA_THETA] += 2 * M_PI;
            }
            
        last_time_pose = current_time.seconds();
        last_pose = *msg;
    }
    
    const int Time_Interval_Millisec = 100; // test for 10, 100, and 1000 ms
    double delta_t_pose = 0.1; // the elapsed time since the last pose message
    double delta_t_control = 0.1; // the delta t for state prediction (the elapsed time since the last control message or the last pose message)
    double last_time_pose = 0.0; // the time of the last pose message
    double last_time_twist = 0.0; // the time of the last twist message
    double last_time_timer = 0.0; // the time of the last timer callback
    turtlesim::msg::Pose last_pose;

    geometry_msgs::msg::Twist last_twist; 
    double delta_pose[3] = {0.0, 0.0, 0.0}; // delta_x, delta_y, delta_theta
    
    enum DeltaPoseIndex {DELTA_X, DELTA_Y, DELTA_THETA};

    double state[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // x, y, theta, vx, vy, vtheta
    //double state[3] = {0.0, 0.0, 0.0}; // x, y, theta
    enum StateOdomIndex {X, Y, THETA, VX, VY, VTHETA};
    double control[2] = {0.0, 0.0}; // v, w
    enum ControlIndex {V, W};      

    const double x0 = 5.5;
    const double y0 = 5.5;
    const double theta0 = 0.0;
    const double vx0 = 0.0;
    const double vy0 = 0.0;
    const double vtheta0 = 0.0;

    double mean[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double covariance[6][6] =     {
        {0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.1, 0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.1, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.1, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.1, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.1}
    };

    double state_turtle2[3] = {0.0, 0.0, 0.0};

    rclcpp::TimerBase::SharedPtr timer{nullptr};
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_pose;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_pose_turtle2; 
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_twist;
    
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client{nullptr};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
    bool spawned = false;    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyKFNode>());
  rclcpp::shutdown();
  return 0;
}
