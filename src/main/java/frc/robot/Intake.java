/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Grizzly Robotics Intake File
 * Handles code to actuate intake
 */
public class Intake {
    private final static TalonSRX intakeMotor = new TalonSRX(Constants.kMotorIntakePort);
    private final static TalonSRX indexStage1Motor = new TalonSRX(Constants.kMotorIndexStage1Port);
    private final static TalonSRX indexStage2Motor = new TalonSRX(Constants.kMotorIndexStage2Port);

    private Joystick operatorController = DriveTrain.operatorController;

    private static boolean manualControl = true;

    public Intake(){
        SmartDashboard.putNumber("Left Trigger", 0);
        SmartDashboard.putNumber("Right Trigger", 0);
    }

    /**
     * Periodic code to update intake for cargo/hatch
     */
    public void updateIntake() {
        double intakeStage1 = operatorController.getRawAxis(Constants.kLeftTrigger);
        double intakeStage2 = operatorController.getRawAxis(Constants.kRightTrigger);

        SmartDashboard.putNumber("Left Trigger", intakeStage1);
        SmartDashboard.putNumber("Right Trigger", intakeStage2);

        if(Math.abs(intakeStage1) >= Constants.kTriggerDeadZone){
            indexStage1Motor.set(ControlMode.PercentOutput, -1);
        } else{
            indexStage1Motor.set(ControlMode.PercentOutput, 0.40);
        }

        if(Math.abs(intakeStage2) >= Constants.kTriggerDeadZone){
            indexStage2Motor.set(ControlMode.PercentOutput, -1);
            intakeMotor.set(ControlMode.PercentOutput, -0.50);
        } else{
            indexStage2Motor.set(ControlMode.PercentOutput, 0);
            intakeMotor.set(ControlMode.PercentOutput, 0);
        }

    }

}
