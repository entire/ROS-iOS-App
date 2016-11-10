//
//  ROSTalkerNode.mm
//  ROSiOS
//
//  Created by FurutaYuki on 1/25/15.
//  Copyright (c) 2015 Furushchev. All rights reserved.
//

#import "ROSTalkerNode.h"
#import <sstream>
#import <std_msgs/String.h>

ROSTalkerNode::ROSTalkerNode()
{
    th_ = new boost::thread(&ROSTalkerNode::spin, this);
    pub_ = nh_.advertise<std_msgs::String>("chatter", 1000);
}

ROSTalkerNode::~ROSTalkerNode()
{
    ros::shutdown();
    th_->join();
    delete th_;
}

void ROSTalkerNode::spin()
{
    ros::spin();
}

std::string ROSTalkerNode::publish(int count)
{
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    
    pub_.publish(msg);
    
    ROS_INFO("%s", msg.data.c_str());
    
    return msg.data;
}
