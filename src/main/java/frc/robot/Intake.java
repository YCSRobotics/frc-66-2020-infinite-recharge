/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
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

    private final static Solenoid intakeSolenoid = new Solenoid(2);
    private final static Solenoid sensorPower = new Solenoid(6);

    private final static DigitalInput intakeSensor1 = new DigitalInput(0);
    private final static DigitalInput intakeSensor2 = new DigitalInput(1);
    private final static DigitalInput stage1Sensor1 = new DigitalInput(2);
    private final static DigitalInput stage1Sensor2 = new DigitalInput(3);
    private final static DigitalInput stage2Sensor1 = new DigitalInput(4);
    private final static DigitalInput stage2Sensor2 = new DigitalInput(5);

    private Joystick operatorController = DriveTrain.operatorController;

    private static boolean manualControl = true;
    private static boolean isIntakeCmd = false;
    private static boolean isSingleShotCmd = false;
    private static boolean isAutoShotCmd = false;

    private static int indexerBallCount = 0;

    boolean intakeSensor = false;
    boolean stage1Sensor = false;
    boolean stage2Sensor = false;


    //Intake States
    final static int IDLE           = 0;
    final static int OVERRIDE       = 1;
    final static int INTAKE         = 2;
    final static int INDEX_STEP1    = 3;
    final static int INDEX_STEP2    = 4;
    final static int INDEX_STEP3    = 5;
    final static int INDEX_STEP4    = 6;
    final static int REVERSE_INDEX  = 7;

    private static int currentIntakeState = IDLE;


    public Intake(){
        indexStage1Motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                                      Constants.kPIDLoopIdx,
                                                      Constants.kTimeoutMs);
        //Set Sensor Phase
        indexStage1Motor.setSensorPhase(false);
        //Config peak and nominal outputs
        indexStage1Motor.configNominalOutputForward(0, Constants.kTimeoutMs);
        indexStage1Motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
        indexStage1Motor.configPeakOutputForward(0.5);
        indexStage1Motor.configPeakOutputReverse(-0.5);
        //Config Velocity closed loop gains
        indexStage1Motor.config_kP(Constants.kPIDLoopIdx, 0.2);
        indexStage1Motor.config_kI(Constants.kPIDLoopIdx, 0);
        indexStage1Motor.config_kD(Constants.kPIDLoopIdx, 0);
        indexStage1Motor.config_kF(Constants.kPIDLoopIdx, 0);

        SmartDashboard.putNumber("Left Trigger", 0);
        SmartDashboard.putNumber("Right Trigger", 0);
    }

    public void initIntake(){
        sensorPower.set(true);
        indexStage1Motor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        indexerBallCount = 0;
    }

    /**
     * Periodic code to update intake for cargo/hatch
     */
    public void updateIntake() {
        double intakeTrigger = operatorController.getRawAxis(Constants.kLeftTrigger);
        double singleShotTrigger = operatorController.getRawAxis(Constants.kRightTrigger);
        boolean isAutoShotCmd = operatorController.getRawButton(Constants.kBButton);

        intakeSensor = intakeSensor1.get();
        stage1Sensor = stage1Sensor1.get();
        stage2Sensor = stage2Sensor1.get();

        SmartDashboard.putNumber("Stage 1 Position", indexStage1Motor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Stage 1 Output", indexStage1Motor.getMotorOutputPercent());
        SmartDashboard.putBoolean("Intake Sensor", intakeSensor);
        SmartDashboard.putBoolean("Stage 1 Sensor", stage1Sensor);

        SmartDashboard.putNumber("Intake State", currentIntakeState);

        if(Math.abs(intakeTrigger) >= Constants.kTriggerDeadZone){
            isIntakeCmd = true;
            intakeSolenoid.set(true);
        } else{
            isIntakeCmd = false;
            intakeSolenoid.set(false);
        }

        if(Math.abs(singleShotTrigger) >= Constants.kTriggerDeadZone){
            isSingleShotCmd = true;
            indexStage2Motor.set(ControlMode.PercentOutput, 1);
        } else{
            isSingleShotCmd = false;
            indexStage2Motor.set(ControlMode.PercentOutput, 0);
        }

        switch(currentIntakeState){
            case IDLE:
                stateActionIdle();
                break;
            case OVERRIDE:
                stateActionOverride();
                break;
            case INTAKE:
                stateActionIntake();
                break;
            case INDEX_STEP1:
                stateActionIndexStep1();
                break;
            case INDEX_STEP2:
                stateActionIndexStep2();
                break;
            case INDEX_STEP3:
                stateActionIndexStep3();
                break;
            case INDEX_STEP4:
                stateActionIndexStep4();
                break;
        }

        SmartDashboard.putNumber("Index Ball Count", indexerBallCount);


    }

    private void stateActionOverride() {
        if(isIntakeCmd){
            indexStage1Motor.set(ControlMode.PercentOutput, -1);
            intakeMotor.set(ControlMode.PercentOutput, -.75);
            intakeSolenoid.set(true);
        } else{
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
            intakeMotor.set(ControlMode.PercentOutput, 0);
            intakeSolenoid.set(false);
        }

        if(isSingleShotCmd){
            indexStage2Motor.set(ControlMode.PercentOutput, 1);
        } else{
            indexStage2Motor.set(ControlMode.PercentOutput, 0);
        }

        if(operatorController.getRawButton(Constants.kLeftBumper)){
            indexStage1Motor.set(ControlMode.PercentOutput, 1);
            indexStage2Motor.set(ControlMode.PercentOutput, -1);
            intakeMotor.set(ControlMode.PercentOutput, 1);
        }

        if(operatorController.getRawButton(Constants.kRightBumper)){
            intakeMotor.set(ControlMode.PercentOutput, 1);
        }
    }

    private void stateActionReverseIndex() {

    }

    private void stateActionIdle() {
        currentIntakeState = OVERRIDE;

        /*if(!isIntakeCmd){
            //No user input, turn off all motors
            intakeMotor.set(ControlMode.PercentOutput, 0);
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
        }else{
            //Intake on but indexer off
            intakeMotor.set(ControlMode.PercentOutput, Constants.kIntakeOnSpeed);
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
            currentIntakeState = INTAKE;
        }*/
    }

    private void stateActionIntake(){
        if(!isIntakeCmd){
            //No user input, turn off all motors
            intakeMotor.set(ControlMode.PercentOutput, 0);
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
            currentIntakeState = IDLE;
        }else if ((intakeSensor)/* &&
                  (!stage1Sensor)*/){
            //Ball sensed, intake and indexer on
            intakeMotor.set(ControlMode.PercentOutput,  Constants.kIntakeOnSpeed);
            indexStage1Motor.set(ControlMode.PercentOutput, Constants.kIndexStage1OnSpeed);
            currentIntakeState = INDEX_STEP1;
        }else{
            //Do Nothing, remain in the Intake State
        }
    }
    private void stateActionIndexStep1(){
        if(!isIntakeCmd){
            //No user input, turn off all motors
            intakeMotor.set(ControlMode.PercentOutput, 0);
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
            currentIntakeState = IDLE;
        }else if((!intakeSensor) /*&&
                 (!stage1Sensor)*/){
            //Ball has moved past the sensor, index to offset
            intakeMotor.set(ControlMode.PercentOutput, Constants.kIntakeOnSpeed);
            indexStage1Motor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
            indexStage1Motor.set(ControlMode.Position, Constants.kIndexStep1Offset);
            currentIntakeState = INDEX_STEP2;
        }else if(stage1Sensor){
            //Ball has reached the end of Stage 1
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
            currentIntakeState = INTAKE;
        }
        else{
            //Do nothing, wait for ball to pass the sensor
        }
    }
    private void stateActionIndexStep2(){
        double indexer_position;

        indexer_position = indexStage1Motor.getSelectedSensorPosition();
        
        if(!isIntakeCmd){
            //No user input, turn off all motors
            intakeMotor.set(ControlMode.PercentOutput, 0);
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
            currentIntakeState = IDLE;
        }else if((Math.abs(indexer_position) > Constants.kIndexStep1Offset * 0.9) ||
                 (stage1Sensor)){
            //Ball has moved to offset OR has reached end of stage 1
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
            currentIntakeState = INDEX_STEP3;
        }else{
            //Do nothing and wait for indexer to reach position
        }
    }

    private void stateActionIndexStep3(){
        if(!isIntakeCmd){
            if(!stage1Sensor){
                currentIntakeState = IDLE;
            }else{
                intakeMotor.set(ControlMode.PercentOutput, 0);
                indexStage1Motor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
                indexStage1Motor.set(ControlMode.Position, Constants.kIndexStep3Offset);
                currentIntakeState = INDEX_STEP4;
            }
        }else{
            if(!stage1Sensor){
                currentIntakeState = INTAKE;
            }else{
                //Do nothing and wait for trigger release
            }
        }
    }

    private void stateActionIndexStep4(){
        double indexer_position;

        indexer_position = indexStage1Motor.getSelectedSensorPosition();

        if(!isIntakeCmd){
            if(Math.abs(indexer_position) > Constants.kIndexStep1Offset * 0.9){
                indexStage1Motor.set(ControlMode.PercentOutput, 0);
                currentIntakeState = INTAKE;
            }else{
                //Do nothing and wait
            }
        }else{
            //Intake button was pressed, so abort and go back to Intake state
            indexStage1Motor.set(ControlMode.PercentOutput, 0);
            currentIntakeState = INTAKE;
        }
    }

}
