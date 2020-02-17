/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;

/**
 * Grizzly Robotics Sensor Data Class
 * Contains static references to grab sensor data
 */
public class SensorData {
    private static AHRS navSensor = new AHRS(SPI.Port.kMXP, (byte) 100);

    private static AnalogInput leftUltraSensor = new AnalogInput(Constants.kLeftUltraSensor);
    private static AnalogInput rightUltraSensor = new AnalogInput(Constants.kRightUltraSensor);

    private static DigitalInput bannerSensor = new DigitalInput(2);

    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("ChickenVision");

    public static void resetYaw() { navSensor.reset(); }

    /**
     * Gets the current robot yaw, left right
     * @return double -180 to 180
     */
    public static double getYaw() {
        return navSensor.getYaw();
    }

    /**
     * Gets the current robot pitch, up down
     * @return double -180 to 180
     */
    public static double getPitch() {
        return navSensor.getPitch();
    }

    /**
     * Gets the current robot roll
     * @return double -180 to 180
     */
    public static double getRoll() {
        return navSensor.getRoll();
    }

    /**
     * Grabs the state of the ball sensor
     * @return true/false
     */
    public static boolean getBallSensorState() { 
        return bannerSensor.get(); 
    }

    /**
     * Grabs the current angle of the vision target, after offset calculations
     * @return double -90 to 90
     */
    public static double angleToVisionTarget() {
        double currentDistance = distanceToVisionTarget();

        double offset = Math.toDegrees(Math.atan(Constants.kVisionOffset/currentDistance));

        double data = table.getEntry("tapeYaw").getDouble(0.0);

        return data - offset;
    }

    /**
     * Grabs the current distance of the vision target from the camera
     * @return double distance in inches
     */
    public static double distanceToVisionTarget() { 
        return table.getEntry("tapeDistance").getDouble(0.0); 
    }

    /**
     * Is the tape detected?
     * @return true/false
     */
    public static boolean tapeDetected() { 
        return table.getEntry("tapeDetected").getBoolean(false); 
    }

    /**
     * Get left infared distance in volts
     * @return double volts
     */
    public static double getLeftIRDistance() {
        return leftUltraSensor.getVoltage();
    }

    /**
     * Get right infared distance in volts
     * @return double voltss
     */
    public static double getRightIRDistance() {
        return rightUltraSensor.getVoltage();
    }
}

