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
    //CAN Talons
    public static final int kMotorLeftMasterPort =      0;
    public static final int kMotorLeftFollowerPort =    1;
    public static final int kMotorRightMasterPort =     2;
    public static final int kMotorRightFollowerPort =   3;
    public static final int kMotorShooter1Port =        4;
    public static final int kMotorShooter2Port =        5;
    public static final int kMotorIndexStage2Port =     6;
    public static final int kMotorIntakePort=           7;
    public static final int kMotorIndexStage1Port =     8;

    //Spark Max (Neos)
    public static final int kMotorShooterAltitude =     1;
    public static final int kMotorShooterAzimuth  =     2;

    //PWM
    public static final int kServoCameraTilt = 0;

    //CANCoders
    public static final int kCANCoderOne = 0;
    public static final int kCANCoderTwo = 1;

    public static final boolean kInvertRightMotor = false;
    public static final boolean kInvertLeftMotor = true;
    public static final int kInvertRightMotorMultiplier = -1;
    public static final double kDriveRampRate = 0.08;

    //encoder constants
    public static final int kEncoderDistancePerRevolution = 4096;
    public static final int kWheelDiameter = 6;
    public static final double kEncoderRotationRate = 5.4;
    public static final double kPi = 3.14159265;
    public static final double kMagMultiplier = ((kEncoderDistancePerRevolution*kEncoderRotationRate)/(kPi * kWheelDiameter));

    public static final double kMotorOutputTreshold = 0.05;
    public static final double kEncoderDelta = 1;
    public static final int kEncoderFaultThreshold = 25; // .5/.02 = 25

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

    //deadzones
    public static final double kDeadZone = 0.05;
    public static final double kTurretDeadZone = 0.2;
    public static final double kTriggerDeadZone = 0.1;
    
    //gains
    public static final double kGyroGain = 0.02;

    //solenoids
    public static final int kShifterSolenoid = 0;
    public static final int kIntakeSolenoid  = 1;

    //drivetrain
    public static final double kSkimGain = 0.15;
    public static final double kDriveSpeed = 0.5;
    public static final double kTurnGain = 0.6;
    public static final double kTurnFinesseGain = 0.5;
    public static final double kTargetZoneOffset = 20; //distance (in inches) to back away from wall
    public static final double kTargetZoneSpeed = 0.5; //Speed to back away from wall in target zone

    public static final double kVisionGain = 0.7;
    public static final double kTurnVisionGain = 0.6;
    public static final double kVisionOffset = 10;

    public static final int kVisionDistanceLimit = 70;

    //intake/indexer
    public static final double kIntakeOnSpeed = -0.75;
    public static final double kIndexStage1OnSpeed = -0.30;

    public static final double kIndexStep1Offset = -3072;
    public static final double kIndexStep3Offset = 3072;

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
    public static final double kShooterAltMotRevsPerDegree = .504;   
    public static final double kShooterAziMotRevsPerDegree = 1.03;
    
    //Camera constants
    public static final double kCameraFoVDeg        = 60; //For Logitech C270
    public static final double kCamera360pYRes      = 240;
    public static final double kCamera360pXRes      = 320;
    public static final double kCameraXDegPerPixel  = kCameraFoVDeg/kCamera360pXRes;
    public static final double kTargetXSize         = 39.25; //inches
    public static final double kTargetYSize         = 17; //inches

    public static final double kCameraHomeAngle     = 120;
    public static final double kCameraPitchDeadZone = 10;

    //Shooter PID gains
    public static final double kP_alt           = 0.1; 
    public static final double kI_alt           = 0;
    public static final double kD_alt           = 0; 
    public static final double kIz_alt          = 0; 
    public static final double kFF_alt          = 0; 
    public static final double kMaxOutput_alt   = .5; 
    public static final double kMinOutput_alt   =-.5;

    public static final double kP_azi           = 0.1; 
    public static final double kI_azi           = 0;
    public static final double kD_azi           = 0; 
    public static final double kIz_azi          = 0; 
    public static final double kFF_azi          = 0; 
    public static final double kMaxOutput_azi   = 1; 
    public static final double kMinOutput_azi   =-1;

    public static final double kP_shtr          = 0.15; 
    public static final double kI_shtr          = 0;
    public static final double kD_shtr          = 4.5; 
    public static final double kIz_shtr         = 0; 
    public static final double kFF_shtr         = 0.0263; 
    public static final double kMaxOutput_shtr  = 1; 
    public static final double kMinOutput_shtr  =-1;
    
    public static final int kPIDLoopIdx      = 0;
    public static final int kTimeoutMs       = 30;
    public static final int kRemoteSensor0   = 0;
    public static final boolean kShooterOneInverted = true;
    public static final boolean kShooterTwoInverted = true;

}
