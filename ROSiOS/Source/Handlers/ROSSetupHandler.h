//
//  ROSSetupHandler.h
//  ROSiOS
//
//  Created by Kosuke Hata on 11/9/16.
//  Copyright © 2016 Canopy Robotics. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface ROSSetupHandler : NSObject

+ (void)insertROSConfigurationAsInitialInterfaceOfWindow:(UIWindow*)window withNodeName:(NSString*)nodeName;
+ (void)insertROSConfigurationAsInitialInterfaceOfWindow:(UIWindow *)window withNodeName:(NSString *)nodeName anonymous:(BOOL)anon;
+ (NSString *)nodeName;
+ (NSString *)getMyIPAddress;
+ (BOOL)isHostValid:(NSString*)host;

@end
