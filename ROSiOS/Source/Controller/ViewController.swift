//
//  ViewController.swift
//  kittyhawk-pfd
//
//  Created by Kosuke Hata on 11/3/16.
//  Copyright © 2016 Canopy Robotics. All rights reserved.
//

import UIKit

class ViewController: UIViewController {

    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        
        self.navigationController?.navigationBar.isHidden = true
        
        let flightView = PrimaryFlightDisplayView(frame: UIScreen.main.bounds)
        flightView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        self.view.addSubview(flightView)
        
        //flightView.setAttitude(rollRadians: Double(1), pitchRadians: Double(1.5))
        //flightView.setHeadingDegree(Double(300))
        //flightView.setAirSpeed(Double(20))
        //flightView.setAltitude(Double(16500))
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
}

