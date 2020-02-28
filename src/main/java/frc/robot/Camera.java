/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles camera interactions
 */
public class Camera {
    public NetworkTableEntry isValid;
    public NetworkTableEntry targetBoundingWidth, targetBoundingHeight;
    public NetworkTableEntry yaw;
    public NetworkTableEntry isDriverMode;

    public double targetDistance = -1;
    public double targetYaw = 0;

    public boolean targetValid;

    NetworkTableInstance table = NetworkTableInstance.getDefault();

    public Camera() {       
    }

    public void updateCamera(){
        NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("TestCam");
        isValid = cameraTable.getEntry("isValid");
        yaw = cameraTable.getEntry("yaw");
        targetBoundingWidth = cameraTable.getEntry("targetBoundingWidth");
        targetBoundingHeight = cameraTable.getEntry("targetBoundingHeight"); 
        
        calculateTargetDistance();
        SmartDashboard.putNumber("Yaw", yaw.getDouble(0.0));
    }

    private void calculateTargetDistance(){
        double base_pixels;
        double base_degrees;

        /********************************************
         *   Calculate distance d to the target
         * 
         *    image size (Constant - in pixels)
         *|--------------------------------------|               
         *          targetBoundingWidth 
         *          (from NetworkTables)        
         *         |----------*----------| -> 39.25 in (actual)
         *          \   base  *
         *           \        *
         *            \       *
         *             \      *
         *              \     * d = base/tan(a)
         *               \    *
         *                \ a *
         *                 \  *
         *                  \ *
         *                   \*
         *                 camera
         ********************************************/

        if(isValid.getBoolean(true)){
            //get the apparent width of the target (in pixels) and divide by 2
            base_pixels = targetBoundingWidth.getDouble(0.0)/2;
            //calculate apparent width of base in degrees by multiplying image by Field of View/image pixels
            base_degrees = Math.toRadians(base_pixels*Constants.kCameraXDegPerPixel);
            //actual width of vision target is known, so calculate distance by using tangent function

            SmartDashboard.putNumber("base degrees", base_degrees);
            targetDistance = (Constants.kTargetXSize/2)/Math.tan(base_degrees);
        }
        else{    
            //Invalid image data
            targetDistance = -1;
        }
        SmartDashboard.putNumber("calculated distance", targetDistance);
    }

    public double getTargetDistance(){
        return(targetDistance);
    }

    public double getTargetYaw(){
        return(targetYaw);
    }

    public boolean isTargetValid(){
        return(targetValid);
    }

}
