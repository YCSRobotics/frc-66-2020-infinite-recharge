/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
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
    private static CANSparkMax shooterAltitudeMotor = new CANSparkMax(Constants.kMotorShooterAltitude, MotorType.kBrushless);
    private static CANSparkMax shooterAzimuthMotor = new CANSparkMax(Constants.kMotorShooterAzimuth, MotorType.kBrushless);

    private CANPIDController m_pidAltCntlr;
    private CANPIDController m_pidAziCntlr;

    //SPARK MAX integrated encoders
    private CANEncoder m_encoderAltCntlr;
    private CANEncoder  m_encoderAziCntlr;

    //CTRE CANCoders
    private static CANCoder canCoderOne = new CANCoder(Constants.kCANCoderOne);
    private static CANCoder canCoderTwo = new CANCoder(Constants.kCANCoderTwo);

    public double kP_alt, kI_alt, kD_alt, kIz_alt, kFF_alt, kMaxOutput_alt, kMinOutput_alt;
    public double kP_azi, kI_azi, kD_azi, kIz_azi, kFF_azi, kMaxOutput_azi, kMinOutput_azi;
    public double kP_shtr, kI_shtr, kD_shtr, kIz_shtr, kFF_shtr, kMaxOutput_shtr, kMinOutput_shtr;

    private static TalonSRX motorShooterOne = new TalonSRX(Constants.kMotorShooter1Port);
    private static TalonSRX motorShooterTwo = new TalonSRX(Constants.kMotorShooter2Port);

    private Joystick operatorController = new Joystick(Constants.kOperatorController);

    private boolean isY_Pressed         = false;
    private boolean isCloseShotEnabled  = false;
    private boolean isA_Pressed         = false;
    private boolean isAutoTargetEnabled = false;

    public Shooter() {
        // Configure Neo 550
        /**
         * The restoreFactoryDefaults method can be used to reset the configuration
         * parameters in the SPARK MAX to their factory default state. If no argument is
         * passed, these parameters will not persist between power cycles
         */
        //shooterAltitudeMotor.restoreFactoryDefaults();

        /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
        m_pidAltCntlr = shooterAltitudeMotor.getPIDController();
        m_pidAziCntlr = shooterAzimuthMotor.getPIDController();

        // Encoder object created to display position values
        m_encoderAltCntlr = shooterAltitudeMotor.getEncoder();
        m_encoderAziCntlr = shooterAzimuthMotor.getEncoder();

        // set PID coefficients
        m_pidAltCntlr.setP(Constants.kP_alt);
        m_pidAltCntlr.setI(Constants.kI_alt);
        m_pidAltCntlr.setD(Constants.kD_alt);
        m_pidAltCntlr.setIZone(Constants.kIz_alt);
        m_pidAltCntlr.setFF(Constants.kFF_alt);
        m_pidAltCntlr.setOutputRange(Constants.kMinOutput_alt, kMaxOutput_alt);

        m_pidAziCntlr.setP(Constants.kP_azi);
        m_pidAziCntlr.setI(Constants.kI_azi);
        m_pidAziCntlr.setD(Constants.kD_azi);
        m_pidAziCntlr.setIZone(Constants.kIz_azi);
        m_pidAziCntlr.setFF(Constants.kFF_azi);
        m_pidAziCntlr.setOutputRange(Constants.kMinOutput_azi, kMaxOutput_azi);

        // display PID coefficients on SmartDashboard
        //displayAltPIDValues();
        //displayAziPIDValues();
        //displayShooterPIDValues();

        SmartDashboard.putNumber("Altitude Set Point", 0);
        SmartDashboard.putNumber("Altitude Position", 0);

        SmartDashboard.putNumber("Shooter 1 Speed",0);
        SmartDashboard.putNumber("Shooter 2 Speed",0);

        SmartDashboard.putNumber("Target Distance", 0);

        /**
         * Parameters can be set by calling the appropriate Set method on the
         * CANSparkMax object whose properties you want to change
         * 
         * Set methods will return one of three CANError values which will let you know
         * if the parameter was successfully set: CANError.kOk CANError.kError
         * CANError.kTimeout
         */
        if (shooterAltitudeMotor.setIdleMode(IdleMode.kCoast) != CANError.kOk) {
            SmartDashboard.putString("Idle Mode", "Error");
        }

        /**
         * Similarly, parameters will have a Get method which allows you to retrieve
         * their values from the controller
         */
        if (shooterAltitudeMotor.getIdleMode() == IdleMode.kCoast) {
            SmartDashboard.putString("Idle Mode", "Coast");
        } else {
            SmartDashboard.putString("Idle Mode", "Brake");
        }

        // Set ramp rate to 0
        if (shooterAltitudeMotor.setOpenLoopRampRate(0) != CANError.kOk) {
            SmartDashboard.putString("Ramp Rate", "Error");
        }

        // read back ramp rate value
        SmartDashboard.putNumber("Ramp Rate", shooterAltitudeMotor.getOpenLoopRampRate());
    }

    

    /**
     * Periodic method to update shooter
     */
    public void updateShooter() {
        double distance;
        int index;

        double turret_output = operatorController.getRawAxis(Constants.kLeftXAxis);

        double target_angle;
        double target_speed;

        distance = getTargetDistance();
        
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        //updateAltPIDGains();
        //updateAziPIDGains();
        //updateShooterPIDGains();

        if((operatorController.getRawButton(Constants.kYButton)) && (!isY_Pressed)){
            //Rising Y edge - Toggle "Close Shot"
            isY_Pressed = true;
            isA_Pressed = false;
            //isCloseShotEnabled = !isCloseShotEnabled;
            //isAutoTargetEnabled = false;
            setShooterAltitude(-30);
        }else if((operatorController.getRawButton(Constants.kAButton)) && (!isA_Pressed)){
            //Rising A edge - Toggle "Auto Target"
            isY_Pressed = false;
            isA_Pressed = true;
            //isCloseShotEnabled = false;
            //isAutoTargetEnabled = !isAutoTargetEnabled;
        }else if ((!operatorController.getRawButton(Constants.kYButton)) && 
                  (!operatorController.getRawButton(Constants.kAButton))){
            isY_Pressed = false;
            isA_Pressed = false;
            setShooterAltitude(-1);
        }else{}
        
        if((isCloseShotEnabled)||(isAutoTargetEnabled)){
            //First check front of goal shot  
            if(isCloseShotEnabled){
                index = 0;
            }else{
            /*A button pressed - Target shot - Determine lookup table index*/

                if((distance > 0) && (distance <= 5)){
                    index = 1;
                } else if ((distance > 5 ) && (distance <= 10)){
                    index = 2;
                } else if ((distance > 10 ) && (distance <= 15)){
                    index = 3;
                } else if ((distance > 15 ) && (distance <= 20)){
                    index = 4;
                } else if ((distance > 20 ) && (distance <= 25)){
                    index = 5;
                } else {
                    index = 6;
                }
            }

            /*Lookup shooter angle and speed*/
            target_angle = ShooterLookup.shooterLookupTable[index][0];
            target_speed = ShooterLookup.shooterLookupTable[index][1];

            /*Set shooter angle*/
            setShooterAltitude(target_angle);

            /*Set shooter speeds*/
            motorShooterOne.set(ControlMode.PercentOutput, target_speed);
            motorShooterTwo.set(ControlMode.PercentOutput,-(target_speed));

        }else{
            /*Set shooter speeds to 0*/
            motorShooterOne.set(ControlMode.PercentOutput, 0);
            motorShooterTwo.set(ControlMode.PercentOutput, 0); 

            /*Set shooter angle*/
            //setShooterAltitude(0);
        }
        
        /*Set Turret output*/
        if(Math.abs(turret_output) > 0.2){
            shooterAzimuthMotor.set(-turret_output);
        }
        else{
            shooterAzimuthMotor.set(0);
        }
        
        SmartDashboard.putBoolean("Close Shot Enabled", isCloseShotEnabled);
        SmartDashboard.putBoolean("Auto Target Enabled", isAutoTargetEnabled);
        SmartDashboard.putNumber("Shooter 1 Speed", canCoderOne.getVelocity());
        SmartDashboard.putNumber("Shooter 2 Speed", canCoderTwo.getVelocity());
        SmartDashboard.putNumber("Altitude Position", m_encoderAltCntlr.getPosition());

        // periodically read voltage, temperature, and applied output and publish to
        // SmartDashboard
        SmartDashboard.putNumber("Voltage", shooterAltitudeMotor.getBusVoltage());
        SmartDashboard.putNumber("Temperature", shooterAltitudeMotor.getMotorTemperature());
        SmartDashboard.putNumber("Output", shooterAltitudeMotor.getAppliedOutput());
    }

    private void updateShooterPIDGains() {
    }

    private void updateAziPIDGains() {
        // read PID coefficients from SmartDashboard
        double p_azi = SmartDashboard.getNumber("Azimuth P Gain", 0);
        double i_azi = SmartDashboard.getNumber("Azimuth I Gain", 0);
        double d_azi = SmartDashboard.getNumber("Azimuth D Gain", 0);
        double iz_azi = SmartDashboard.getNumber("Azimuth I Zone", 0);
        double ff_azi = SmartDashboard.getNumber("Azimuth Feed Forward", 0);
        double max_azi = SmartDashboard.getNumber("Azimuth Max Output", 0);
        double min_azi = SmartDashboard.getNumber("Azimuth Min Output", 0);
        
        if((p_azi != kP_azi)) { m_pidAziCntlr.setP(p_azi); kP_azi = p_azi; }
        if((i_azi != kI_azi)) { m_pidAziCntlr.setI(i_azi); kI_azi = i_azi; }
        if((d_azi != kD_azi)) { m_pidAziCntlr.setD(d_azi); kD_azi = d_azi; }
        if((iz_azi != kIz_azi)) { m_pidAziCntlr.setIZone(iz_azi); kIz_azi = iz_azi; }
        if((ff_azi != kFF_azi)) { m_pidAziCntlr.setFF(ff_azi); kFF_azi = ff_azi; }
        if((max_azi != kMaxOutput_azi) || (min_azi != kMinOutput_azi)) { 
            m_pidAziCntlr.setOutputRange(min_azi, max_azi); 
            kMinOutput_azi = min_azi; kMaxOutput_azi = max_azi; 
        }
    }

    private void updateAltPIDGains() {
        // read PID coefficients from SmartDashboard
        double p_alt = SmartDashboard.getNumber("Altitude P Gain", 0);
        double i_alt = SmartDashboard.getNumber("Altitude I Gain", 0);
        double d_alt = SmartDashboard.getNumber("Altitude D Gain", 0);
        double iz_alt = SmartDashboard.getNumber("Altitude I Zone", 0);
        double ff_alt = SmartDashboard.getNumber("Altitude Feed Forward", 0);
        double max_alt = SmartDashboard.getNumber("Altitude Max Output", 0);
        double min_alt = SmartDashboard.getNumber("Altitude Min Output", 0);

        if((p_alt != kP_alt)) { m_pidAltCntlr.setP(p_alt); kP_alt = p_alt; }
        if((i_alt != kI_alt)) { m_pidAltCntlr.setI(i_alt); kI_alt = i_alt; }
        if((d_alt != kD_alt)) { m_pidAltCntlr.setD(d_alt); kD_alt = d_alt; }
        if((iz_alt != kIz_alt)) { m_pidAltCntlr.setIZone(iz_alt); kIz_alt = iz_alt; }
        if((ff_alt != kFF_alt)) { m_pidAltCntlr.setFF(ff_alt); kFF_alt = ff_alt; }
        if((max_alt != kMaxOutput_alt) || (min_alt != kMinOutput_alt)) { 
            m_pidAltCntlr.setOutputRange(min_alt, max_alt); 
            kMinOutput_alt = min_alt; kMaxOutput_alt = max_alt; 
        }
    }

    private double getTargetDistance() {
        double distance;
        distance = SmartDashboard.getNumber("Target Distance", 0);
        return(distance);
    }

    private void setShooterAltitude(double angle){
        double motor_revs; 
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
        
         /*Convert Shooter Angle to motor revolutions*/
        motor_revs = angle * Constants.kShooterAltMotRevsPerDegree;

        SmartDashboard.putNumber("Altitude Set Point", motor_revs);

        m_pidAltCntlr.setReference(motor_revs, ControlType.kPosition);
    }

    private void setShooterAzimuth(double angle){
        double motor_revs; 
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
        
         /*Convert Shooter Angle to motor revolutions*/
        motor_revs = angle * Constants.kShooterAziMotRevsPerDegree;

        SmartDashboard.putNumber("Azimuth SetPoint", motor_revs);

        m_pidAziCntlr.setReference(motor_revs, ControlType.kPosition);
    } 

    public double getShooterMotor1Speed(){
        //returns degrees/second
        return(canCoderOne.getVelocity());
    }

    private void displayShooterPIDValues() {
        SmartDashboard.putNumber("Shooter P Gain", kP_shtr);
        SmartDashboard.putNumber("Shooter I Gain", kI_shtr);
        SmartDashboard.putNumber("Shooter D Gain", kD_shtr);
        SmartDashboard.putNumber("Shooter I Zone", kIz_shtr);
        SmartDashboard.putNumber("Shooter Feed Forward", kFF_shtr);
        SmartDashboard.putNumber("Shooter Max Output", kMaxOutput_shtr);
        SmartDashboard.putNumber("Shooter Min Output", kMinOutput_shtr);
        SmartDashboard.putNumber("Shooter Set Rotations", 0);
    }

    private void displayAziPIDValues() {
        SmartDashboard.putNumber("Azimuth P Gain", kP_azi);
        SmartDashboard.putNumber("Azimuth I Gain", kI_azi);
        SmartDashboard.putNumber("Azimuth D Gain", kD_azi);
        SmartDashboard.putNumber("Azimuth I Zone", kIz_azi);
        SmartDashboard.putNumber("Azimuth Feed Forward", kFF_azi);
        SmartDashboard.putNumber("Azimuth Max Output", kMaxOutput_azi);
        SmartDashboard.putNumber("Azimuth Min Output", kMinOutput_azi);
        SmartDashboard.putNumber("Azimuth Set Rotations", 0);
    }

    private void displayAltPIDValues() {
        SmartDashboard.putNumber("Altitude P Gain", kP_alt);
        SmartDashboard.putNumber("Altitude I Gain", kI_alt);
        SmartDashboard.putNumber("Altitude D Gain", kD_alt);
        SmartDashboard.putNumber("Altitude I Zone", kIz_alt);
        SmartDashboard.putNumber("Altitude Feed Forward", kFF_alt);
        SmartDashboard.putNumber("Altitude Max Output", kMaxOutput_alt);
        SmartDashboard.putNumber("Altitude Min Output", kMinOutput_alt);
        SmartDashboard.putNumber("Altitude Set Rotations", 0);
    }

}
