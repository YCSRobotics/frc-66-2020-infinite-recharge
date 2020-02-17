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
    private final static TalonSRX intakeMotor = new TalonSRX(Constants.kIntakeMotor);
    private Joystick operatorController = DriveTrain.operatorController;

    private final static Solenoid gearIntakeBoi = new Solenoid(Constants.kGearIntakeSolenoid);

    private static boolean manualControl = true;

    /**
     * Periodic code to update intake for cargo/hatch
     */
    public void updateIntake() {
        double intakeIn = operatorController.getRawAxis(Constants.kLeftTrigger);
        double intakeOut = operatorController.getRawAxis(Constants.kRightTrigger);

        boolean slowModeActive = SensorData.getBallSensorState();

        if (operatorController.getRawButton(Constants.kLeftBumper)){
            gearIntakeBoi.set(true);

        } else {
            gearIntakeBoi.set(false); }


        if (manualControl) {

            double intakeValue = 0.0;
            double intakeValueSlow = -0.10;

            if (intakeIn > 0.05) {
                intakeValue = -intakeIn;
                intakeValueSlow = slowModeActive ? intakeValue : intakeValue * Constants.kIntakeSlowMaxSpeed;
            }

            if (intakeOut > 0.05) {
                intakeValue = intakeOut;
                intakeValueSlow = intakeValue;
            }

            if (!SensorData.getBallSensorState()) {
                intakeValueSlow *= 0.2;
            }

            intakeMotor.set(ControlMode.PercentOutput, intakeValueSlow);
        }

    }

    /**
     * Sets the wheel intake speed
     * @param power -1.0 to 1.0
     */
    public static void setIntakeState(double power) {
        intakeMotor.set(ControlMode.PercentOutput, power);

    }

    /**
     * Sets the hatch claw grabber to on/off
     * @param state on/off
     */
    public static void setHatchState(boolean state) {
        gearIntakeBoi.set(state);
    }

    public static void setControlState(boolean manualControlEnabled) {
        manualControl = manualControlEnabled;
    }

}
