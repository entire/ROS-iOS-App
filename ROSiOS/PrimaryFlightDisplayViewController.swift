//
//  PrimaryFlightDisplayViewController.swift
//  ROSiOS
//
//  Created by Kosuke Hata on 11/29/16.
//  Copyright © 2016 Furushchev. All rights reserved.
//

import UIKit

@objc class PrimaryFlightDisplayViewController: UIViewController {
    
    override func viewDidLoad() {
        super.viewDidLoad()
        // Do any additional setup after loading the view, typically from a nib.
        
        let flightView = PrimaryFlightDisplayView(frame: UIScreen.main.bounds)
        flightView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        self.view.addSubview(flightView)
        
        self.navigationItem.title = "Flight Display"
        
        self.navigationItem.leftBarButtonItem = UIBarButtonItem(title: "close", style: .plain, target: self, action: #selector(self.close))
        
        //flightView.setAttitude(rollRadians: Double(1), pitchRadians: Double(1.5))
        //flightView.setHeadingDegree(Double(300))
        //flightView.setAirSpeed(Double(20))
        //flightView.setAltitude(Double(16500))
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
    func close () {
        self.dismiss(animated: true, completion: nil)
    }
    
}


