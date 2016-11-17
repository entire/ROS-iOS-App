//
//  TopicListTableViewCell.h
//  ROS-Display
//
//  Created by FurutaYuki on 12/27/14.
//  Copyright (c) 2016 Canopy Robotics All rights reserved.
//

#import <UIKit/UIKit.h>

@interface TopicListTableViewCell : UITableViewCell
@property (weak, nonatomic) IBOutlet UILabel *topicName;
@property (weak, nonatomic) IBOutlet UILabel *dataType;

@end
