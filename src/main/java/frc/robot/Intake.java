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

/**
 * Grizzly Robotics Intake File
 * Handles code to actuate intake
 */
public class Intake {
    private final static TalonSRX intakeMotor = new TalonSRX(Constants.kIntakeMotorPort);
    private final static TalonSRX indexStage1Motor = new TalonSRX(Constants.kMotorIndexStage1Port);
    private final static TalonSRX indexStage2Motor = new TalonSRX(Constants.kMotorIndexStage2Port);

    private Joystick operatorController = DriveTrain.operatorController;

    private static boolean manualControl = true;

    /**
     * Periodic code to update intake for cargo/hatch
     */
    public void updateIntake() {
        double intakeIn = operatorController.getRawAxis(Constants.kLeftTrigger);
        double intakeOut = operatorController.getRawAxis(Constants.kRightTrigger);

        if(Math.abs(intakeIn) >= Constants.kTriggerDeadZone){
            indexStage1Motor.set(ControlMode.PercentOutput, 1);
        } else{
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
        }

        if(Math.abs(intakeOut) >= Constants.kTriggerDeadZone){
            indexStage2Motor.set(ControlMode.PercentOutput, 1);
        } else{
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
        }

    }

}
