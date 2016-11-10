//
//  ROSListenerNode.h
//  ROSiOS
//
//  Created by FurutaYuki on 1/25/15.
//  Copyright (c) 2015 Furushchev. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <ros/ros.h>
#import <std_msgs/String.h>

#import <boost/thread/thread.hpp>

#import "ListenerViewController.h"

class ROSListenerNode {
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    
    boost::thread *th_;
public:
    ListenerViewController __weak *listenerViewController;
    
    ROSListenerNode();
    ~ROSListenerNode();
    
    void spin();
    void chatterCB(const std_msgs::String::ConstPtr&);
};
