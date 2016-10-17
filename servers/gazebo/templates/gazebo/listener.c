/**
 * \author Kunal Tyagi
 * \file listener.c
 * \brief template for future plugin files
 */

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>

// Function is called everytime a message is received.
void cb(ConstWorldStatisticsPtr &_msg)
{
    // Dump the message contents to stdout.
    std::cout << _msg->DebugString();
}

int main()
{
    // Initialize transport
    gazebo::transport::init();
    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());

    node->Init();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);
    // Start transport
    gazebo::transport::run();

    // Busy wait loop...replace with your own code as needed.
    while (true)
    {    
        gazebo::common::Time::MSleep(10);
    }

    // Make sure to shut everything down.
    gazebo::transport::fini();
}

