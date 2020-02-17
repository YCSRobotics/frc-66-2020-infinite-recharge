/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Grizzly Robotics Lift Class
 * Handles controlling the robot fourbar
 */
public class FourBarControl {
    private static TalonSRX fourBarMotorMaster = new TalonSRX(Constants.kFourBarMotor);

    private Joystick operatorController = DriveTrain.operatorController;

    private double fourBarPosition = 0.0;
    public static double setPosition = 0.0;

    private static boolean override = false;

    public FourBarControl() {
        //configure sensor boiz
		fourBarMotorMaster.setSensorPhase(true);
		fourBarMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		fourBarMotorMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        fourBarMotorMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);

        //Zero sensor position on initialization
        fourBarMotorMaster.setSelectedSensorPosition(0);

        fourBarMotorMaster.selectProfileSlot(0, 0);
		fourBarMotorMaster.config_kP(0, 4, 10);
		fourBarMotorMaster.config_kI(0, 0, 10);
		fourBarMotorMaster.config_kD(0, 400, 10);
        fourBarMotorMaster.config_kF(0, 0, 10);

        //max outputs
        fourBarMotorMaster.configPeakOutputForward(Constants.kFourBarMaxForward);
        fourBarMotorMaster.configPeakOutputReverse(Constants.kFourBarMaxReverse);

        fourBarMotorMaster.configOpenloopRamp(Constants.kFourBarRamp);
        fourBarMotorMaster.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Periodic method to update fourbar
     */
    public void updateFourBar() {
        if (override) {
            //maintain current position
            fourBarMotorMaster.set(ControlMode.Position, setPosition);
            return;
        }

        double fourBarThrottle = -operatorController.getRawAxis(Constants.kRightYAxis);
        SmartDashboard.putNumber("Current Fourbar Output", fourBarMotorMaster.getMotorOutputPercent());
        SmartDashboard.putNumber("Operator Fourbar", fourBarThrottle);
        SmartDashboard.putNumber("Four Bar Set Position", fourBarPosition);

        fourBarPosition = getFourBarPosition();

        //implement fake arbitrary feedforward depending on power
        if(fourBarThrottle > Constants.kFourBarDeadZone) {
            fourBarMotorMaster.config_kF(0, 1, 10);

        } else if(fourBarThrottle < -Constants.kFourBarDeadZone) {
            fourBarMotorMaster.config_kF(0, 0, 10);

        }

        //lift fourbar to position and hold when no more motor output is being applied
        //manual fourbar contrl
        if (Math.abs(fourBarThrottle) > Constants.kFourBarDeadZone) {
            setPosition = fourBarPosition;
            fourBarMotorMaster.set(ControlMode.PercentOutput, fourBarThrottle);

        } else if (operatorController.getRawButton(Constants.kAButton)) {
            setPosition = Constants.kFourBarPos1;
            fourBarMotorMaster.set(ControlMode.Position, setPosition);

        } else if (operatorController.getRawButton(Constants.kBButton)) {
            setPosition = Constants.kFourBarPos2;
            fourBarMotorMaster.set(ControlMode.Position, setPosition);

        } else if (operatorController.getRawButton(Constants.kYButton)) {
            setPosition = Constants.kFourBarPos3;
            fourBarMotorMaster.set(ControlMode.Position, setPosition);

        } else {
            fourBarMotorMaster.set(ControlMode.Position, setPosition);
        }

    }


    public static double getFourBarPosition() {
        return fourBarMotorMaster.getSelectedSensorPosition(0);
    }

    public static void setOverride(boolean state) {
        override = state;
    }


}
