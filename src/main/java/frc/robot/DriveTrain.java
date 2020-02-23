/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Grizzly Robotics Drivetrain
 * Robot Movement
 */
public class DriveTrain {
    //initialize drivetrain stuff
    private static TalonSRX leftMaster = new TalonSRX(Constants.kMotorLeftMasterPort);
    private static TalonSRX leftFollower = new TalonSRX(Constants.kMotorLeftFollowerPort);

    private static TalonSRX rightFollower = new TalonSRX(Constants.kMotorRightFollowerPort);
    private static TalonSRX rightMaster = new TalonSRX(Constants.kMotorRightMasterPort);

    public static Joystick driverController = new Joystick(Constants.kDriverController);
    public static Joystick operatorController = new Joystick(Constants.kOperatorController);

    private static Solenoid speedyMode = new Solenoid(Constants.kShifterSolenoid);

    private static Timer timer = new Timer();

    public static boolean autonomousActive = false;

    private static boolean invertMode = false;

    //driver states
    private static boolean isYawZeroed = false;
    private static boolean isMovingDistance = false;
    private static boolean isTurning = false;
    private static boolean isFollowingTarget = false;

    public static double throttleValue;
    public static double turnValue;

    private static double targetDistance;
    private static double turnAngle;

    private static double leftOutput;
    private static double rightOutput;

    //autonomous specific
    private static double cancellationPeriod = 0.0;
    private static boolean isTargetFound = false;
    private static boolean visionCancellation = false;

    private static boolean override = false;

    public DriveTrain(){
        //set robot invert state
        rightMaster.setInverted(Constants.kInvertRightMotor);
        rightFollower.setInverted(Constants.kInvertRightMotor);

        leftMaster.setInverted(Constants.kInvertLeftMotor);
        leftFollower.setInverted(Constants.kInvertLeftMotor);

        //tell the followers to begin following the master
        leftFollower.set(ControlMode.Follower, leftMaster.getDeviceID());
        rightFollower.set(ControlMode.Follower, rightMaster.getDeviceID());

        //initialize our ctre mag encoders
        leftMaster.setSensorPhase(false);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        rightMaster.setSensorPhase(true);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        leftMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);

        //our "acceleration/decceleration"
        leftMaster.configOpenloopRamp(Constants.kDriveRampRate);
        rightMaster.configOpenloopRamp(Constants.kDriveRampRate);

        timer.start();
    }

    /** 
     * Periodic update drivetrain loop
     */
    public void updateDrivetrain() {
        boolean shiftState = driverController.getRawAxis(Constants.kRightTrigger) > 0;
        boolean invertButtonPressed = driverController.getRawButton(Constants.kSelectButton);

        SmartDashboard.putNumber("turnValue", turnValue);

        if (override) {
            calculateMotorOutputs(throttleValue, turnValue);
            setMotorOutput(leftOutput, rightOutput);
            return;
        }

        //activate our shifting gearboxes
        if (shiftState) {
            setSpeedyMode(true);
        } else {
            setSpeedyMode(false);
        }

        //autonomousActive is an 'ABSOLUTE' override, this prevents auto code from "leaking" into teleop
        if (((isMovingDistance) || (isTurning) || (isFollowingTarget)) && autonomousActive) {
            driveAutonomous();

        //if go straight button is pressed and yaw is not zero, then zero
        } else if ((driverController.getRawButton(Constants.kAButton)) && (!isYawZeroed)) {
            //wait one loop to zero to prevent garbage
            SensorData.resetYaw();
            isYawZeroed = true;

        //if go straight button pressed, go straight
        } else if (driverController.getRawButton(Constants.kAButton)) {
            goStraight();

        //auto align button based off of vision, and assuming the target is found
        } else if ((driverController.getRawButton(Constants.kYButton)) && (!isTargetFound) && (!isYawZeroed)) {
            SensorData.resetYaw();
            isYawZeroed = true;

        //no target found, acts as a go straight
        } else if ((driverController.getRawButton(Constants.kYButton)) && (!isTargetFound)) {
            goStraight();

        //no hybrid-autonomous modes selected, normal drivetrain controls
        } else {
            throttleValue = getThrottleInput();//Scaled Throttle Input
            turnValue = getTurnInput();//Scaled Turn Input

            //disable throttle
            if (Math.abs(throttleValue) > Constants.kDeadZone) {
                enableDrivetrainDynamicBraking(false);
            }

            //set this to zero if not using it, to rezero at beginning of hybrid-auto loop
            isYawZeroed = false;

            //update isTargetFound only once, prevents garbage and handles our vision fallbacks
            isTargetFound = SensorData.tapeDetected();

        }

        //calculates left and right outputs
        calculateMotorOutputs(throttleValue, turnValue);

        //invert mode boiz
        if (invertButtonPressed && timer.get() > 0.5) {
            invertMode = !invertMode;
            timer.reset();

        }

        //check if invert mode is active, and invert
        if ((invertMode) && (!isMovingDistance) && (!isTurning)) {
            leftOutput = -leftOutput;
            rightOutput = -rightOutput;

            var tempVar = leftOutput;

            //switch the sides
            leftOutput = rightOutput;
            rightOutput = tempVar;

        }

        //raw set motor output function
        setMotorOutput(leftOutput, rightOutput);
    }

    public static void setOverride(boolean state) {
        override = state;
    }

    /**
     * Handles go straight via vision target, has default parameters for autonomous control.
     * Has a cancellation period for autonomous, where if it loses the target it has a "safety" period.
     */
    private static void alignUsingVision() {
        if (autonomousActive) {
            throttleValue = Constants.kVisionPower;
        } else {
            throttleValue = getThrottleInput() * 0.4;
        }

        enableDrivetrainDynamicBraking(false);

        var targetAngleVision = SensorData.angleToVisionTarget();

        double targetHeading = SensorData.angleToVisionTarget();

        //cancellation timer is active, then activate and wait
        if (visionCancellation) {
            if (timer.get() > cancellationPeriod + 0.5) {
                AutoRoutine.isWithinTargetRange = false;
                isFollowingTarget = false;

            } else if (SensorData.tapeDetected()) {
                visionCancellation = false;

            } else {
                return;

            }
        }

        //if tape is not detected, activate go straight if in hybrid-auto, activate cancellation timer in full-auto
        if (!SensorData.tapeDetected()) {
            //stop running autonomous, lost target
            //lost target go straight in manual control
            isTargetFound = false;

            cancellationPeriod = timer.get();

            visionCancellation = true;

            return;

        }

    }

    //trims the output, due to math possibly giving us over 1 outputs
    private double trim(double input) {
        if (input > 1.0) {
            return -((input - 1.0) * Constants.kSkimGain);

        } else if (input < -1.0) {
            return -((input + 1.0) * Constants.kSkimGain);

        } return 0;

    }

    //activates shifting gearbox solenoid
    public static void setSpeedyMode(boolean state) {
        speedyMode.set(state);
    }

    /**
     * Retrieves throttle input with specified limits
     * @return double throttle input -1.0 to 1.0
     */
    public static double getThrottleInput() {
        double forwardValue = driverController.getRawAxis(Constants.kLeftYAxis);
        boolean speedyMode = driverController.getRawAxis(Constants.kLeftTrigger) > 0;

        if (Math.abs(forwardValue) < Constants.kDeadZone) {
            return 0;
        }

     if(!speedyMode) {
            forwardValue = forwardValue * Constants.kDriveSpeed;

        } else {
            //
        }

        return (Math.abs(forwardValue) > Constants.kDeadZone ? -forwardValue : 0.0);
    }

    /**
     * Retrives turn input with specified limits
     * @return double turn input -1.0 to 1.0
     */
    public static double getTurnInput() {
        //Returns scaled turn input
        double turnValue = driverController.getRawAxis(Constants.kRightXAxis);

        if (Math.abs(getThrottleInput()) > Constants.kDeadZone) {
            turnValue = turnValue * Constants.kTurnGain;

        } else {
            turnValue = turnValue * Constants.kTurnFinesseGain;
        }

        return turnValue;
    }

    /**
     * Converts output to arcade style from tank style
     * @param throttle -1.0 to 1.0 forward value
     * @param turn -1.0 to 1.0 turn value
     */
    private void calculateMotorOutputs(double throttle, double turn){
        //being overriden by climber
        leftOutput = throttle + turn;
        rightOutput = throttle - turn;

        //apply our skim gains to smooth turning
        leftOutput = leftOutput + trim(rightOutput);
        rightOutput = rightOutput + trim(leftOutput);
    }

    /**
     * Set motor output directly, bypassing limits
     * @param leftMotorValue raw left motor output
     * @param rightMotorValue raw right motor output
     */
    public static void setMotorOutput(double leftMotorValue, double rightMotorValue) {
        leftMaster.set(ControlMode.PercentOutput, leftMotorValue);
        rightMaster.set(ControlMode.PercentOutput, rightMotorValue);
    }

    //Go straight using gyro only
    public static void goStraight() {
        //sets throttle value based on throttle input and turn value based on heading error
        enableDrivetrainDynamicBraking(false);
        throttleValue = getThrottleInput();
        turnValue = (0 - SensorData.getYaw() ) * Constants.kGyroGain;
    }

    //sets the target for autonomous motion
    public static void setMoveDistance(double distance, double throttle){
        SensorData.resetYaw();

        leftMaster.setSelectedSensorPosition(0, 0, 0);
        rightMaster.setSelectedSensorPosition(0, 0, 0);


        targetDistance = distance;

        enableDrivetrainDynamicBraking(true);

        if(Math.abs(targetDistance) > Constants.kTargetDistanceThreshold) {
            isMovingDistance = true;
            throttleValue = throttle;
        }
        else {
            isMovingDistance = false;
            throttleValue = 0.0;
        }
    }
    
    //set drivetrain variables to begin following via vision
    public static void moveToVisionTarget(double throttle) {
        enableDrivetrainDynamicBraking(true);

        throttleValue = throttle;

        isFollowingTarget = true;
        AutoRoutine.isWithinTargetRange = false;

    }

    /**
     * Turn to the target autonomously
     */
    public static void setTurnToTarget(double turn_power, double angle){
        SensorData.resetYaw();
        
        isTurning = true;
		turnAngle = Math.abs(angle);
		turnValue = turn_power;
    }
    
    //handles movement of robot during autonomous
    public static void driveAutonomous(){
        double distance_error;
        
        //if moving forward
		if(isMovingDistance){
			//Move distance without tracking vision target
            //distance_error = targetDistance - getAverageDistance();
            distance_error = targetDistance - getLeftWheelDistance();
			
			//Check Distance
			if((targetDistance > 0) && 
			   (distance_error <= Constants.kTargetDistanceThreshold)){
				//Robot has reached target
				throttleValue = 0.0;
				isMovingDistance = false;

			} else if((targetDistance <= 0) && (distance_error >= -Constants.kTargetDistanceThreshold)){
				//Robot has reached target
				throttleValue = 0.0;
				isMovingDistance = false;

			} else {
				//Have not reached target
			}

			turnValue = (0 - SensorData.getYaw() ) * Constants.kGyroGain;

        //if turning
		} else if (isTurning) {
            if (Math.abs(SensorData.getYaw()) >= turnAngle) {
                throttleValue = 0.0;
                turnValue = 0.0;
                isTurning = false;

            } else {
                //Do Nothing while turning
            }

        } else if (isFollowingTarget) {
            alignUsingVision();

		} else {
			//No Auton move in progress
			throttleValue = 0.0;
			turnValue = 0.0;
		}
    }

    /**
     * Enables/disables active braking
     * @param enable true/false
     */
    public static void enableDrivetrainDynamicBraking(boolean enable){
		if(enable){
			leftMaster.setNeutralMode(NeutralMode.Brake);
			leftFollower.setNeutralMode(NeutralMode.Brake);
			rightMaster.setNeutralMode(NeutralMode.Brake);
			rightFollower.setNeutralMode(NeutralMode.Brake);
		} else{
			leftMaster.setNeutralMode(NeutralMode.Coast);
			leftFollower.setNeutralMode(NeutralMode.Coast);
			rightMaster.setNeutralMode(NeutralMode.Coast);
			rightFollower.setNeutralMode(NeutralMode.Coast);
		}
		
    }

    //gets average distance
    public static double getAverageDistance() {
        return (getLeftWheelDistance() + getRightWheelDistance()) / 2;
    }

    //reset left encoder
    public static void resetLeftWheelEncoder() {
        leftMaster.setSelectedSensorPosition(0, 0, 0);
    }

    //reset right encoder
    public static void resetRightWheelEncoder() {
        rightMaster.setSelectedSensorPosition(0, 0, 0);
    }

    /**
     * Gets the left wheel position from encoder
     * @return Raw Left Wheel Position in [Counts]
     */
    public static int getLeftWheelPosition() {
        return leftMaster.getSelectedSensorPosition();
    }

    /**
     * Gets the right wheel position from encoder
     * @return Raw Right Wheel Position in [Counts]
     */
    public static int getRightWheelPosition() {
        return rightMaster.getSelectedSensorPosition() * Constants.kInvertRightMotorMultiplier;
    }

    /**
     * Gets the left wheel distance from encoder
     * @return Raw Left Wheel Distance in inches
     */
    public static double getLeftWheelDistance() {
        return leftMaster.getSelectedSensorPosition() / Constants.kMagMultiplier;
    }

    /**
     * Gets the right wheel distance from encoder
     * @return Raw Left Wheel Distance in inches
     */
    public static double getRightWheelDistance() {
        return (rightMaster.getSelectedSensorPosition() / Constants.kMagMultiplier) * Constants.kInvertRightMotorMultiplier;
    }

    /**
     * Is the robot moving forward?
     * @return true if moving, false if not
     */
    public static boolean isMovingDistance(){
        return(isMovingDistance);
    }

    /**
     * Is the robot turning?
     * @return true if turning, false if not
     */
    public static boolean isTurning(){
        return(isTurning);
    }

    /**
     * Is the robot following the vision target?
     * @return true if following, false if not
     */
    public static boolean isFollowingTarget() {
        return (isFollowingTarget);
    }    

}
