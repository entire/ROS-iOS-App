//
//  ROSSetupHandler.m
//  ROSiOS
//
//  Created by Kosuke Hata on 11/9/16.
//  Copyright © 2016 Canopy Robotics. All rights reserved.
//

#import "ROSSetupHandler.h"
#import <ros/init.h>
#import <ros/master.h>
#import <ifaddrs.h>
#import <arpa/inet.h>
#import "UIAlertController+window.h"

@implementation ROSSetupHandler

+ (void)insertROSConfigurationAsInitialInterfaceOfWindow:(UIWindow*)window withNodeName:(NSString*)nodeName
{
    [ROSSetupHandler insertROSConfigurationAsInitialInterfaceOfWindow:window withNodeName:nodeName anonymous:NO];
}
    
+ (void)insertROSConfigurationAsInitialInterfaceOfWindow:(UIWindow *)window withNodeName:(NSString *)nodeName anonymous:(BOOL)anon
{
    NSUserDefaults *ud = [NSUserDefaults standardUserDefaults];
    [ud setObject:nodeName forKey:@"kROSiOSNodeNameKey"]; // DO NOT CHANGE!
    [ud setBool:anon forKey:@"kROSiOSNodeAnonymous"]; // DO NOT CHANGE!
    [ud synchronize];
}

+ (NSString *)nodeName {
    NSUserDefaults *ud = [NSUserDefaults standardUserDefaults];
    NSString *nodename = [ud objectForKey:@"kROSiOSNodeNameKey"];
    NSString *targetNodename;
    if (nodename) {
        targetNodename = nodename;
    } else {
        targetNodename = @"ros_ios_app";
    }
    
    return targetNodename;
}

+ (BOOL)connectToROSMaster:(NSString *)targetURI
{
    if ([ROSSetupHandler isHostValid:targetURI]) {
        NSString * master_uri = [@"ROS_MASTER_URI=" stringByAppendingString:targetURI];
        NSLog(@"ROS_MASTER URI: %@",master_uri);
        
        NSString * ip = [ROSSetupHandler getMyIPAddress];
        NSString * hostname = [@"ROS_HOSTNAME=" stringByAppendingString:ip];
        NSLog(@"ROS_HOSTNAME: %@",hostname);
        
        putenv((char *)[master_uri UTF8String]);
        putenv((char *)[hostname UTF8String]);
        putenv((char *)"ROS_HOME=/tmp");
        
        int argc = 0;
        char **argv = NULL;
        NSString *nodeName = [ROSSetupHandler nodeName];

        if(!ros::isInitialized())
        {
            if ([[ROSSetupHandler nodeName] isEqualToString:@"ros_ios_app"]) {
                ros::init(argc, argv, [nodeName UTF8String], ros::init_options::AnonymousName);
            } else {
                ros::init(argc, argv, [nodeName UTF8String]);
            }
        }
        else
        {
            NSLog(@"ROS already initialised. Can't change the ROS_MASTER_URI");
        }
        
        if(ros::master::check())
        {
            NSLog(@"Connected to the ROS master !");
        }
        else {
            UIAlertController *alert = [UIAlertController alertControllerWithTitle:@"Error!" message:@"Couldn't join the ROS master" preferredStyle:UIAlertControllerStyleAlert];
            [alert addAction:[UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleCancel handler:nil]];
            [alert show];
            return NO;
        }
    } else {
        UIAlertController *alert = [UIAlertController alertControllerWithTitle:@"Error!" message:@"Invalid ROS Master URI" preferredStyle:UIAlertControllerStyleAlert];
        [alert addAction:[UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleCancel handler:nil]];
        [alert show];
        return NO;
    }
    
    return YES;
}

#pragma mark - Private IPConfigurationUtilities Methods

+ (NSString *)getMyIPAddress
{
    struct ifaddrs *interfaces = NULL;
    struct ifaddrs *temp_addr = NULL;
    NSString *wifiAddress = nil;
    NSString *cellAddress = nil;
    
    // retrieve the current interfaces - returns 0 on success
    if(!getifaddrs(&interfaces)) {
        // Loop through linked list of interfaces
        temp_addr = interfaces;
        while(temp_addr != NULL) {
            sa_family_t sa_type = temp_addr->ifa_addr->sa_family;
            if(sa_type == AF_INET || sa_type == AF_INET6) {
                NSString *name = [NSString stringWithUTF8String:temp_addr->ifa_name];
                NSString *addr = [NSString stringWithUTF8String:inet_ntoa(((struct sockaddr_in *)temp_addr->ifa_addr)->sin_addr)]; // pdp_ip0
                NSLog(@"NAME: \"%@\" addr: %@", name, addr); // see for yourself
                
                if([name hasPrefix:@"en"]) {
                    // Interface is the wifi connection on the iPhone
                    if (![addr hasPrefix:@"0."])
                        wifiAddress = addr;
                } else {
                    if([name isEqualToString:@"pdp_ip0"]) {
                        // Interface is the cell connection on the iPhone
                        cellAddress = addr;
                    }
                }
            }
            temp_addr = temp_addr->ifa_next;
        }
        // Free memory
        freeifaddrs(interfaces);
    }
    NSString *addr = wifiAddress ? wifiAddress : cellAddress;
    return addr ? addr : @"0.0.0.0";
}

+ (BOOL)isHostValid:(NSString*)host
{
    NSURL* url = [NSURL URLWithString:host];
    if ([url scheme] && [url host] && [url port]) {
        return YES;
    } else {
        return NO;
    }
}



@end

