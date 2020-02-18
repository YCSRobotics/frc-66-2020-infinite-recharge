/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

/**
 * Grizzly Robotics Shooter Class Handles controlling the robot Shooter
 */
public class Shooter {
    /**
     * deviceID is the CAN ID of the SPARK MAX you are using. Change to match your
     * setup
     */
    private static CANSparkMax shooterMotorMaster = new CANSparkMax(Constants.kNeoDeviceID, MotorType.kBrushless);

    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    private static WPI_TalonSRX motorShooterOne = new WPI_TalonSRX(Constants.kMotorShooter1Port);
    private static WPI_TalonSRX motorShooterTwo = new WPI_TalonSRX(Constants.kMotorShooter2Port);

    private Joystick operatorController = new Joystick(Constants.kOperatorController);

    // private double fourBarPosition = 0.0;
    // public static double setPosition = 0.0;

    // private static boolean override = false;

    public Shooter() {
        // Configure Neo 550
        /**
         * The restoreFactoryDefaults method can be used to reset the configuration
         * parameters in the SPARK MAX to their factory default state. If no argument is
         * passed, these parameters will not persist between power cycles
         */
        shooterMotorMaster.restoreFactoryDefaults();

        /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
        m_pidController = shooterMotorMaster.getPIDController();

        // Encoder object created to display position values
        m_encoder = shooterMotorMaster.getEncoder();

         // PID coefficients
        kP = 0.1; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);

        /**
         * Parameters can be set by calling the appropriate Set method on the
         * CANSparkMax object whose properties you want to change
         * 
         * Set methods will return one of three CANError values which will let you know
         * if the parameter was successfully set: CANError.kOk CANError.kError
         * CANError.kTimeout
         */
        if (shooterMotorMaster.setIdleMode(IdleMode.kCoast) != CANError.kOk) {
            SmartDashboard.putString("Idle Mode", "Error");
        }

        /**
         * Similarly, parameters will have a Get method which allows you to retrieve
         * their values from the controller
         */
        if (shooterMotorMaster.getIdleMode() == IdleMode.kCoast) {
            SmartDashboard.putString("Idle Mode", "Coast");
        } else {
            SmartDashboard.putString("Idle Mode", "Brake");
        }

        // Set ramp rate to 0
        if (shooterMotorMaster.setOpenLoopRampRate(0) != CANError.kOk) {
            SmartDashboard.putString("Ramp Rate", "Error");
        }

        // read back ramp rate value
        SmartDashboard.putNumber("Ramp Rate", shooterMotorMaster.getOpenLoopRampRate());
    }

    /**
     * Periodic method to update shooter
     */
    public void updateShooter() {
        int distance;
        int index;

        double motor_revs;

        int target_angle;
        int target_speed;

        distance = getTargetDistance();

        /*Determine lookup table index*/
        if(distance <= 5){
            index = 0;
        } else if ((distance > 5 ) && (distance <= 10)){
            index = 1;
        } else if ((distance > 10 ) && (distance <= 15)){
            index = 2;
        } else if ((distance > 15 ) && (distance <= 20)){
            index = 3;
        } else if ((distance > 20 ) && (distance <= 25)){
            index = 4;
        } else {
            index = 5;
        }

        /*Lookup shooter angle and speed*/
        target_angle = ShooterLookup.shooterLookupTable[index][0];
        target_speed = ShooterLookup.shooterLookupTable[index][1];

        /*Convert Shooter Angle to motor revolutions*/
        motor_revs = target_angle * Constants.kShooterAltMotRevsPerDegree;

        /*Set shooter angle*/
        // Set motor output to joystick value
        //shooterMotorMaster.set(operatorController.getY());

        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }
        
         /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.ControlType.kDutyCycle
         *  com.revrobotics.ControlType.kPosition
         *  com.revrobotics.ControlType.kVelocity
         *  com.revrobotics.ControlType.kVoltage
         */
        m_pidController.setReference(rotations, ControlType.kPosition);
        
        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());

        // periodically read voltage, temperature, and applied output and publish to
        // SmartDashboard
        SmartDashboard.putNumber("Voltage", shooterMotorMaster.getBusVoltage());
        SmartDashboard.putNumber("Temperature", shooterMotorMaster.getMotorTemperature());
        SmartDashboard.putNumber("Output", shooterMotorMaster.getAppliedOutput());

    }

    // public static double getFourBarPosition() {
    //     return fourBarMotorMaster.getSelectedSensorPosition(0);
    // }

    // public static void setOverride(boolean state) {
    //     override = state;
    // }

    private int getTargetDistance(){

        //TODO: add code for getting camera distance
        return(0);
    }

    private void setShooterAzimuth(double target_angle){

    }

}
