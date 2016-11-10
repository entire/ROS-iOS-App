//
//  TalkerViewController.mm
//  ROSiOS
//
//  Created by FurutaYuki on 1/25/15.
//  Copyright (c) 2015 Furushchev. All rights reserved.
//

#import "TalkerViewController.h"
#import "ROSTalkerNode.h"

@interface TalkerViewController ()
{
    ROSTalkerNode *rosTalkerNode_;
    NSTimer *timer_;
    int count_;
}

@property (weak, nonatomic) IBOutlet UITextView *textView;

@end

@implementation TalkerViewController

- (void)viewDidLoad {
    [super viewDidLoad];
}

- (void)viewWillAppear:(BOOL)animated
{
    [super viewWillAppear:animated];
    
    self.textView.text = @"";

    count_ = 0;
    
    rosTalkerNode_ = new ROSTalkerNode();
    
    timer_ = [NSTimer timerWithTimeInterval:0.1f
                                     target:self
                                   selector:@selector(talk)
                                   userInfo:nil
                                    repeats:YES];
    [[NSRunLoop mainRunLoop] addTimer:timer_ forMode:NSDefaultRunLoopMode];
}

- (void)viewWillDisappear:(BOOL)animated
{
    if ([timer_ isValid]) {
        [timer_ invalidate];
        timer_ = nil;
    }
    delete rosTalkerNode_;
    [super viewWillDisappear:animated];
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
}

- (void)talk
{
    std::string s = rosTalkerNode_->publish(count_++);
    self.textView.text = [self.textView.text stringByAppendingFormat:@"%s\n", s.c_str()];
}

@end
