/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Grizzly Robotics Constant File
 * Constant variable names should be in Hungarian notation
 * Example: int kDriveLeftMotorMaster = 0;
 * Hungarian notation variables start with k
 */
public class Constants {
    //motor constants
    public static final int kLeftMotorFollower = 1;
    public static final int kLeftMotorMaster = 0;
    public static final int kRightMotorMaster = 2;
    public static final int kRightMotorFollower = 3;
    public static final int kElevatorMotor = 4;
    public static final int kFourBarMotor = 5;
    public static final int kIntakeMotor = 6;
    public static final int kClimberMotor = 7;
    public static final int kPullMotor = 8;

    public static final boolean kInvertRightMotor = false;
    public static final boolean kInvertLeftMotor = true;
    public static final int kInvertRightMotorMultiplier = -1;
    public static final double kDriveRampRate = 0.08;

    public static final int kMotorLeftMasterPort =      0;
    public static final int kMotorLeftFollowerPort =    1;
    public static final int kMotorRightMasterPort =     2;
    public static final int kMotorRightFollowerPort =   3;
    public static final int kMotorShooter1Port =        4;
    public static final int kMotorShooter2Port =        5;


    //encoder constants
    public static final int kEncoderDistancePerRevolution = 4096;
    public static final int kWheelDiameter = 6;
    public static final double kEncoderRotationRate = 5.4;
    public static final double kPi = 3.14159265;
    public static final double kMagMultiplier = ((kEncoderDistancePerRevolution*kEncoderRotationRate)/(kPi * kWheelDiameter));

    public static final double kMotorOutputTreshold = 0.05;
    public static final double kEncoderDelta = 1;
    public static final int kEncoderFaultThreshold = 25; // .5/.02 = 25

    public static final int kLeftUltraSensor = 0;
    public static final int kRightUltraSensor = 1;

     // Neo550 constants
     public static final int kNeoDeviceID = 1;

    //joystick constants
    public final static int kDriverController = 0;
    public final static int kOperatorController = 1;

    public static final int kLeftXAxis = 0;
    public static final int kLeftYAxis = 1;
    public static final int kRightXAxis = 4;
    public static final int kRightYAxis = 5;
    public static final int kLeftTrigger = 2;
    public static final int kRightTrigger = 3;

    public static final int kAButton = 1;
    public static final int kBButton = 2;
    public static final int kXButton = 3;
    public static final int kYButton = 4;
    public static final int kLeftBumper = 5;
    public static final int kRightBumper = 6;
    public static final int kSelectButton = 7;
    public static final int kStartButton = 8;

    //drive and fourbar deadzones
    public static final double kDeadZone = 0.05;
    public static final double kFourBarDeadZone = 0.2;

    //gains
    public static final double kGyroGain = 0.02;

    //solenoids
    public static final int kShifterSolenoid = 0;
    public static final int kGearIntakeSolenoid = 1;
    public static final int kClimberSolenoid = 2;
    public static final double kIntakeSlowMaxSpeed = 0.1;

    //elevator
    public static final double kElevatorDriveFinesseLimit = 20000;
    public static final double kElevatorDriveMaxSpeed = 0.5;
    public static final double kElevatorOpenRamp = 0.3;
    public static final double kElevatorClosedRamp = 0.1;
    public static final int kElevatorPos1 = 50;
    public static final int kElevatorPos2 = 14000;
    public static final int kElevatorPos3 = 10000;
    public static final int kElevatorPosOffset = 50;

    //fourbar
    public static final double kFourBarRamp = 0.3;
    public static final double kFourBarMaxForward = 0.6;
    public static final double kFourBarMaxReverse = -0.49;

    public static final int kFourBarPos1 = 150;
    public static final int kFourBarPos2 = 275;
    public static final int kFourBarPos3 = 1380;
    public static final int kFourBarPosOffset = 50;

    //drivetrain
    public static final double kSkimGain = 0.15;
    public static final double kDriveSpeed = 0.5;
    public static final double kTurnGain = 0.6;
    public static final double kTurnFinesseGain = 0.5;

    public static final double kVisionGain = 0.7;
    public static final double kTurnVisionGain = 0.6;
    public static final double kVisionOffset = 10;

    public static final int kVisionDistanceLimit = 70;

    //autonomous constants
    public static final int kCenterGoStraightInitDistance = 140;
    public static final double kCenterGoStraightInitPower = 0.4;

    public static final int kRocketInitDistance = 50;
    public static final double kRocketInitPower = 0.42;

    public static final double kVisionPower = 0.3;

    public static final int kRocketTurnAngle = 30;
    public static final double kRocketTurnPower = 0.3;

    public static final double kRocketSecondDistance = 50;
    public static final double kRocketSecondPower = 0.3;

    public static final int kBackupDistance = 30;
    public static final double kBackupPower = 0.5;

    public static final int kRammingDistance = 30;
    public static final double kRammingPower = 0.3;

    public static final int kRocketDeadReckoningDistance = 60;
    public static final double kRocketDeadReckoningPower = 0.3;

    public static final String kVisionCam = "10.0.66.12:1181/stream.mjpg";
    public static final double kTargetDistanceThreshold = 6.0;

    //sensor constants
    public static final double kInfaredRange = 0.35;
    public static final double kInfaredAlignGain = 0.6;

    //Shooter constants
    public static final double kShooterAltMotRevsPerDegree = 1.95; // -> 360*(100:1 VP x 1.95:1 gear drive)    
}
