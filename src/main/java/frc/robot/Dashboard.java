/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Grizzly Robotics Dashboard Class - Handles creation of Shuffleboard UI
 */
public class Dashboard {

    //driverdisplay tab keys
    public static ShuffleboardTab driverDisplayTab = Shuffleboard.getTab("DriverDisplay");

    //diagnostics tab keys
    public static ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("Diagnostics");

    private NetworkTableEntry leftWheelDistanceKey = diagnosticsTab
                                    .add("Left Wheel Distance", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();    
                                    
    private NetworkTableEntry rightWheelDistanceKey = diagnosticsTab
                                    .add("Right Wheel Distance", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();      

    private NetworkTableEntry navYawKey = diagnosticsTab
                                    .add("NavX Yaw", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();      

    private NetworkTableEntry navRollKey = diagnosticsTab
                                    .add("NavX Roll", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();
                                    
    private NetworkTableEntry navPitchKey = diagnosticsTab
                                    .add("NavX Pitch", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry leftMotorPosition = diagnosticsTab
                                    .add("Left Motor Position", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry rightMotorPosition = diagnosticsTab
                                    .add("Right Motor Position", 0.0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry crossBoxSensor = diagnosticsTab
                                    .add("Cross Bar Sensor", false)
                                    .withWidget("Boolean Box")
                                    .getEntry();

    private NetworkTableEntry elevatorPosition = diagnosticsTab
                                    .add("Elevator Position", 0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry fourBarPosition = diagnosticsTab
                                    .add("Fourbar Position", 0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry tapeYaw = diagnosticsTab
                                    .add("Tape Yaw", 0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry tapeDetected = diagnosticsTab
                                    .add("Tape Detected", 0)
                                    .withWidget("Boolean Box")
                                    .getEntry();

    private NetworkTableEntry tapeDistance = diagnosticsTab
                                    .add("Tape Distance", 0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry leftUltraDistance = diagnosticsTab
                                    .add("Left Ultrasonic Distance", 0)
                                    .withWidget("Text View")
                                    .getEntry();

    private NetworkTableEntry rightUltraDistance = diagnosticsTab
                                    .add("Right Ultrasonic Distance", 0)
                                    .withWidget("Text View")
                                    .getEntry();

    public static NetworkTableEntry climberReady = driverDisplayTab
                                    .add("Climber Ready?", false)
                                    .withWidget("Boolean Box")
                                    .withSize(2, 1)
                                    .withPosition(0, 1)
                                    .getEntry();

    public static NetworkTableEntry matchTime = driverDisplayTab
                                    .add("Current Match Time", 0.0)
                                    .withWidget("Text Box")
                                    .withSize(2, 1)
                                    .withPosition(0, 2)
                                    .getEntry();

    public static NetworkTableEntry wenchPosition = driverDisplayTab
            .add("Wench Position", 0.0)
            .withWidget("Text Box")
            .getEntry();

    public static NetworkTableEntry bottomLimitSwitch = driverDisplayTab
            .add("Bottom Limit Switch", false)
            .withWidget("Text Box")
            .getEntry();


    public static NetworkTableEntry climberLimitSwitchForward = driverDisplayTab
            .add("Climber Limit Forward", false)
            .withWidget("Text Box")
            .getEntry();

    public static NetworkTableEntry climberLimitSwitchBack = driverDisplayTab
            .add("Climber Limit Back", false)
            .withWidget("Text Box")
            .getEntry();


    public void updateDiagDashboard() {
        Shuffleboard.selectTab("DriverDisplay");

        climberLimitSwitchForward.setBoolean(Climber.backWenchMotor.getSensorCollection().isFwdLimitSwitchClosed());
        climberLimitSwitchBack.setBoolean(Climber.backWenchMotor.getSensorCollection().isRevLimitSwitchClosed());

        matchTime.setNumber(DriverStation.getInstance().getMatchTime());
        navYawKey.setNumber(SensorData.getYaw());
        navPitchKey.setNumber(SensorData.getPitch());
        navRollKey.setNumber(SensorData.getRoll());

        leftWheelDistanceKey.setNumber(DriveTrain.getLeftWheelDistance());
        rightWheelDistanceKey.setNumber(DriveTrain.getRightWheelDistance());

        leftMotorPosition.setNumber(DriveTrain.getLeftWheelPosition());
        rightMotorPosition.setNumber(DriveTrain.getRightWheelPosition());

        crossBoxSensor.setBoolean(SensorData.getBallSensorState());
        elevatorPosition.setNumber(ElevatorControl.getLiftPosition());
        fourBarPosition.setNumber(FourBarControl.getFourBarPosition());

        tapeDetected.setBoolean(SensorData.tapeDetected());
        tapeDistance.setNumber(SensorData.distanceToVisionTarget());
        tapeYaw.setNumber(SensorData.angleToVisionTarget());

        leftUltraDistance.setNumber(SensorData.getLeftIRDistance());
        rightUltraDistance.setNumber(SensorData.getRightIRDistance());

        wenchPosition.setNumber(Climber.getWenchPosition());
        bottomLimitSwitch.setBoolean(ElevatorControl.bottomLimit());

    }
}
