//
// Created by guillaume on 18/09/18.
//

#ifndef AHAND_SAFETY_H
#define AHAND_SAFETY_H

#include <ros/ros.h>
#include <chrono>

class Safety{

public:

    Safety(){
        max_torque=0.6;
        min_torque=-0.6;
        last_time_message_received = std::chrono::system_clock::now();
        max_time_cmd_delay_seconds = 0.1;
        command_received=false;
        command_received_prev=false;
        count=0;
    }

    void check_time(){

        time_difference = std::chrono::system_clock::now() - last_time_message_received;
        command_received = time_difference.count() < max_time_cmd_delay_seconds;

        if(command_received != command_received_prev){
            if(command_received){
                ROS_WARN_STREAM("Command received: " << time_difference.count());
            }else{
                ROS_WARN_STREAM("NO Command received: " << time_difference.count());
            }
        }
        command_received_prev = command_received;
    }

    void check_torque(double& torque){
        if(!command_received){
            torque=0.0;
            return;
        }

        if(torque > max_torque){
            ROS_WARN_STREAM_THROTTLE(1.0, "torque: " << torque << " exceeds max: " << max_torque << " setting torque to zero!");
            torque=max_torque;
        }
        if(torque < min_torque){
            ROS_WARN_STREAM_THROTTLE(1.0, "torque: " << torque << " exceeds min: " << min_torque << " setting torque to zero!");
            torque=min_torque;
        }
    }

    void update_time(){
        last_time_message_received = std::chrono::system_clock::now();
    }

    private:

    double max_torque;
    double min_torque;
    std::chrono::time_point<std::chrono::system_clock> last_time_message_received;
    std::chrono::duration<double> time_difference;
    double max_time_cmd_delay_seconds;
    bool command_received;
    bool command_received_prev;
    std::size_t count;


};


#endif //AHAND_SAFETY_H
