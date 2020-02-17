package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    public static TalonSRX backWenchMotor = new TalonSRX(Constants.kClimberMotor);
    private static TalonSRX frontPullMotor = new TalonSRX(Constants.kPullMotor);

    private static Solenoid climberSolenoid = new Solenoid(Constants.kClimberSolenoid);

    private static Joystick driverJoystick = DriveTrain.driverController;

    private static final int kElevatorClimbOffset = 1000;

    private static int backDriveDistance = 14000;
    private static final int kBackDriveDistanceSecond = 7000;

    private static final int kDeployRange = 600;

    private static double wenchPosition = 0.0;
    private static double elevatorPosition = 0.0;

    private static boolean hasDeployed = false;
    private static boolean stopElevator = false;

    private static boolean climberActive = false;
    private static boolean wenchDisabled = false;

    //rates for level three
    private static double kClimberIncrementValueLevel3 = 50;
    private static double kElevatorIncrementValueLevel3 = 95;

    //rates for level two
    private static double kClimberIncrementValueLevel2 = 50;
    private static double kElevatorIncrementValueLevel2 = 225;

    private static int kClimbMaxPosition = 26000;
    private static final int kClimbMaxPositionSecond = 18200;
    private static final int kClimberDeployPosition = 16000;

    private static double climbValue = 50;
    private static double elevatorValue = 50;

    private static boolean stopElevatorTrigger = false;
    private static double kElevatorBackDistance = 3000;


    public Climber() {
        backWenchMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        frontPullMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        //Zero sensor position on initialization
        backWenchMotor.setSelectedSensorPosition(0);

        backWenchMotor.setSensorPhase(true);

        backWenchMotor.selectProfileSlot(0, 0);
        backWenchMotor.config_kP(0, 1.2, 10);
        backWenchMotor.config_kI(0, 0, 10);
        backWenchMotor.config_kD(0, 0, 10);
        backWenchMotor.config_kF(0, 0, 10);
    }

    public void updateClimber() {
        //climber should not be deployed before 30s
        if (DriverStation.getInstance().getMatchTime() > 45) {
            Dashboard.climberReady.setBoolean(false);
            return;

        } else {
            Dashboard.climberReady.setBoolean(true);

        }

        var throttle = driverJoystick.getRawAxis(Constants.kLeftYAxis);
        var climbLevelTwo = driverJoystick.getRawButton(Constants.kRightBumper);
        var climbLevelThree = driverJoystick.getRawButton(Constants.kLeftBumper);

        handleDrivetrain(throttle);

        //activate climber for level 3
        //set our various values, and then update positions
        if (climbLevelThree && !climberActive) {
            climbValue = kClimberIncrementValueLevel3;
            elevatorValue = kElevatorIncrementValueLevel3;
            climberActive = true;

            enableClimber();

            return;

        //activate climber for level 2
        } else if (climbLevelTwo && !climberActive) {
            climbValue = kClimberIncrementValueLevel2;
            elevatorValue = kElevatorIncrementValueLevel2;
            kClimbMaxPosition = kClimbMaxPositionSecond;
            backDriveDistance = kBackDriveDistanceSecond;
            climberActive = true;

            enableClimber();

            return;
        }

        if (climberActive) {
            //deploy our climber
            climberSolenoid.set(true);
            FourBarControl.setPosition = 450;
        }

        //are we deployed, and should we disable elevator control?
        if ((backWenchMotor.getSelectedSensorPosition() > wenchPosition - kDeployRange) &&
           ((climbLevelTwo || climbLevelThree) && !hasDeployed)) {
            ElevatorControl.setOverride(true);
            enableClimber();

            hasDeployed = true;

        //user hasn't pressed wench button again and/or we haven't hit our deploy target
        } else if (!hasDeployed) {
            backWenchMotor.set(ControlMode.Position, wenchPosition);
            return;

        } else {
            //continue, we have deployed
        }

        //debug information
        SmartDashboard.putNumber("Elevator Current Position: " , ElevatorControl.getLiftPosition());
        SmartDashboard.putNumber("Elevator Set Position", elevatorPosition);
        SmartDashboard.putNumber("Wench Power", wenchPosition);

        //disables the elevator if limit switch is hit, back off a limit bit
        if ((ElevatorControl.bottomLimit() && !stopElevator)) {
            elevatorPosition = ElevatorControl.getLiftPosition() + 80;
            stopElevator = true;

        //disable the elevator if wench hits target
        } else if ((backWenchMotor.getSelectedSensorPosition() > kClimbMaxPosition - kElevatorClimbOffset) && !stopElevator) {
            stopElevator = true;

        } else {
            //continue with our code
        }

        //back off the wench
        if (driverJoystick.getPOV() == 0 && !wenchDisabled) {
            wenchPosition = wenchPosition - backDriveDistance;
            wenchDisabled = !wenchDisabled;
            elevatorPosition = ElevatorControl.getLiftPosition() + kElevatorBackDistance;

        } else {
            //back off was already pressed, or never pressed
        }

        //if button is pressed and we aren't on our wench target
        if ((climbLevelTwo || climbLevelThree) && backWenchMotor.getSelectedSensorPosition() < kClimbMaxPosition) {
            //only go if we have deployed, prevent the values from skyrocketing waiting for deploy to finish
            if (hasDeployed) {
                backWenchMotor.set(ControlMode.Position, wenchPosition += climbValue);

                if (!stopElevator) {
                    ElevatorControl.setLiftPosition(elevatorPosition -= elevatorValue);
                } else {
                    ElevatorControl.setLiftPosition(elevatorPosition);
                }
            }

        } else {
            //hold position control
            backWenchMotor.set(ControlMode.Position, wenchPosition);
            ElevatorControl.setLiftPosition(elevatorPosition);
        }
    }

    public static double getWenchPosition() {
        return backWenchMotor.getSelectedSensorPosition(0);
    }

    private static void enableClimber() {
        System.out.println("Set override!");

        climberActive = true;

        wenchPosition = kClimberDeployPosition;
        elevatorPosition = ElevatorControl.getLiftPosition();

        //disable robot outputs
        FourBarControl.setOverride(true);
        DriveTrain.setOverride(true);
        DriveTrain.turnValue = 0;

    }

    private void handleDrivetrain(double throttle) {
        if (Math.abs(throttle) > Constants.kDeadZone) {
            frontPullMotor.set(ControlMode.PercentOutput, -throttle);
            DriveTrain.throttleValue = -(throttle * Constants.kDriveSpeed);
        } else {
            DriveTrain.throttleValue = 0;
            frontPullMotor.set(ControlMode.PercentOutput, 0);
        }
    }
}
