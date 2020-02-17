/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;

/**
 * Handles camera interactions
 */
public class Camera {
    public Camera() {
        //don't initialize camera server in initialization
        //causes crashes
        if (RobotBase.isSimulation()) {
            return;
        }

        //create a "secondary" camera server to publish to shuffleboard
        AxisCamera camera = CameraServer.getInstance().addAxisCamera("DriverCamera", Constants.kVisionCam);

        //add camera to display
        Dashboard.driverDisplayTab.add(camera).withSize(6,4).withPosition(2,0);

    }

}
