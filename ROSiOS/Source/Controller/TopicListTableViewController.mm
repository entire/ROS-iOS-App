//
//  TopicListTableViewController.mm
//  ROSiOSSampleApp
//
//  Created by FurutaYuki on 12/27/14.
//  Copyright (c) 2016 Canopy Group Holdings LLC. All rights reserved.
//

#import "TopicListTableViewController.h"
#import "TopicListTableViewCell.h"
#import <ros/master.h>

#define kTopicListTableViewCellIdentifier @"topicListTableViewCellIdentifier"

@interface TopicListTableViewController ()

@property (nonatomic, strong) NSMutableArray *topicNames;
@property (nonatomic, strong) NSMutableArray *dataTypes;
@property (strong, nonatomic) IBOutlet UITableView *tableView;

@end

@implementation TopicListTableViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    
    self.topicNames = [NSMutableArray array];
    self.dataTypes = [NSMutableArray array];
    
    [self getTopics];
}

- (void)getTopics
{
    ros::master::V_TopicInfo topicsInfo;
    ros::master::getTopics(topicsInfo);
    
    for (NSInteger i = 0; i < topicsInfo.size(); ++i) {
        [self.topicNames addObject:[NSString stringWithUTF8String:topicsInfo[i].name.c_str()]];
        [self.dataTypes addObject:[NSString stringWithUTF8String:topicsInfo[i].datatype.c_str()]];
    }
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

#pragma mark - Table view data source

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView {
    return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section {
    return [self.topicNames count];
}


- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath {
    
//    NSLog(@"tableView: %@", [tableView description]);
    
    TopicListTableViewCell *cell = (TopicListTableViewCell*)[self.tableView dequeueReusableCellWithIdentifier:kTopicListTableViewCellIdentifier forIndexPath:indexPath];
    
    if (!cell) {
        cell = [[TopicListTableViewCell alloc] initWithStyle:UITableViewCellStyleSubtitle reuseIdentifier:kTopicListTableViewCellIdentifier];
    }
    
    cell.topicName.text = self.topicNames[[indexPath row]];
    cell.dataType.text = self.dataTypes[[indexPath row]];
    
    return cell;
}

- (void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath
{
    
}

@end
