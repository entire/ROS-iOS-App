//
//  ListenerViewController.mm
//  ROSiOS
//
//  Created by FurutaYuki on 1/25/15.
//  Copyright (c) 2015 Furushchev. All rights reserved.
//

#import "ListenerViewController.h"
#import "ROSListenerNode.h"

@interface ListenerViewController ()
{
    ROSListenerNode *rosListenerNode;
}

@property (weak, nonatomic) IBOutlet UITextView *textView;

@end

@implementation ListenerViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewWillAppear:(BOOL)animated
{
    [super viewWillAppear:animated];
    rosListenerNode = new ROSListenerNode();
    rosListenerNode->listenerViewController = self;
}

- (void)viewWillDisappear:(BOOL)animated
{
    delete rosListenerNode;
    [super viewWillDisappear:animated];
}

- (void)chatterCallback:(NSString*)message
{
    self.textView.text = [self.textView.text stringByAppendingFormat:@"I heard: [%@]\n", message];
}

@end
