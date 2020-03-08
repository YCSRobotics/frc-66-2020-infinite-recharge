package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoRoutine {
    //Timer for timed delays
    private Timer timer = new Timer();

    //Autonomous Routines
    final static int DO_NOTHING           = 0;
    final static int MOVE_ONLY            = 1;
    final static int AUTO_LINE_SHOOT      = 2;
    final static int KESSEL_RUN           = 3;

    //Autonomous States
    final static int START                = 0;
    final static int START_DELAY          = 1;
    final static int SHOOT_DELAY          = 2;
    final static int SHOOT                = 3;
    final static int MOVE_DISTANCE        = 4;
    final static int STOP                 = 255;

    public static double alarmTime;

    public static int selectedAutonRoutine;
    public static int currentAutonState = START;
    public static boolean isWithinTargetRange = false;

    public AutoRoutine(){
        SmartDashboard.putNumber("Auton State", currentAutonState);
        SmartDashboard.putNumber("Selected Auto Routine", selectedAutonRoutine);
    }

    public void setSelectedAutonRoutine(int routine){
        selectedAutonRoutine = routine;
    }

    public void initAutoRoutine(){
        currentAutonState = START;
    }

    /**
     * Autonomous State Machine
     */
    public void updateAutoRoutine(){
        SmartDashboard.putNumber("Selected Auto Routine", selectedAutonRoutine);
        SmartDashboard.putNumber("Current Auto Routine", currentAutonState);
        switch(currentAutonState){
            case START:
                stateActionStart();
                break;
            case START_DELAY:
                stateActionStartDelay();
                break;
            case SHOOT_DELAY:
                stateActionShootDelay();
                break;
            case SHOOT:
                stateActionShoot();
                break;
            case MOVE_DISTANCE:
                stateActionMoveDistance();
                break;
            case STOP:
            default:
                stateActionStop();
            }

            SmartDashboard.putNumber("Auton State", currentAutonState);
    }

    private void stateActionStartDelay() {
        if (timer.get() >= alarmTime){
            setAutonDelay(1);
            Shooter.setAutoShotEnabled(true);
            currentAutonState = SHOOT_DELAY;
        }else{
            //Wait for timer to expire
        }

    }

    private void stateActionShootDelay() {
        if (timer.get() >= alarmTime){
            //Shooter.setAutoShotEnabled();
            Intake.setStage1Speed(-1);
            setAutonDelay(5);
            currentAutonState = SHOOT;
        }else{
            //Wait for timer to expire
        }

    }

    private void stateActionShoot() {
        if(timer.get() >=alarmTime){
            Shooter.setAutoShotEnabled(false);
            Intake.setIntakeSpeed(0);
            Intake.setStage1Speed(0);
            Intake.setStage2Speed(0);
            DriveTrain.setMoveDistance(20, 0.5);
            currentAutonState = MOVE_DISTANCE;
        }
        else{
            //Do nothing and wait for timeout
        }
    }

    // called at the beginning of every autonomous method
    private void stateActionStart(){
        if(selectedAutonRoutine != DO_NOTHING){
            if(selectedAutonRoutine == MOVE_ONLY){
                DriveTrain.setMoveDistance(-20, -.25);
                currentAutonState = MOVE_DISTANCE;
            }else if ((selectedAutonRoutine == AUTO_LINE_SHOOT)){
                Shooter.setAutoShotEnabled(true);
                Shooter.setAutoTurretEnabled(true);
                Intake.setStage2Speed(-1);
                setAutonDelay(0.5);
                currentAutonState = START_DELAY;
            }else{}
        } else{
            //go straight to stop if the selected auton routine was do nothing
            currentAutonState = STOP;
        }
    }

    private void stateActionMoveDistance(){

        if(!DriveTrain.isMovingDistance()) {
            if(selectedAutonRoutine == MOVE_ONLY){
                currentAutonState = STOP;
            }
        } else{
            //Wait for move to complete
        }
    }

    private void stateActionStop(){
        //allow coast after stop
        DriveTrain.autonomousActive = false;
        DriveTrain.setMoveDistance(0.0, 0.0);
    }

    private void setAutonDelay(double delay){
        //Method to make delay timer generic
        timer.reset();
        timer.start();
        alarmTime = timer.get() + delay;
    }

}
