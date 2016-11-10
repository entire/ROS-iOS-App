//
//  ROSSetupHandler.m
//  ROSiOS
//
//  Created by Kosuke Hata on 11/9/16.
//  Copyright © 2016 Canopy Robotics. All rights reserved.
//

#import "ROSSetupHandler.h"

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
    
@end
