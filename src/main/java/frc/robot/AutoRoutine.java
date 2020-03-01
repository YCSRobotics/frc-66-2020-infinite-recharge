package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoRoutine {
    //Timer for timed delays
    private Timer timer = new Timer();

    //Autonomous Routines
    final static int DO_NOTHING           = 0;

    //Autonomous States
    final static int START                        = 0;
    final static int MOVE_DISTANCE                = 1;
    final static int TURN_LEFT                    = 2;
    final static int TURN_RIGHT                   = 3;
    final static int MOVE_VISION_TARGET           = 4;
    final static int AUTO_TURN_DELAY              = 5;
    final static int RELEASE_HATCH                = 6;
    final static int MOVE_DISTANCE_TARGET         = 7;
    final static int MOVE_DISTANCE_ROCKET         = 8;
    final static int BACK_TARGET                  = 9;
    final static int STOP                         = 255;

    public static double alarmTime;

    public static int selectedAutonRoutine;
    public static int currentAutonState = START;
    public static boolean isWithinTargetRange = false;

    public void setSelectedAutonRoutine(int routine){
        selectedAutonRoutine = routine;
    }

    /**
     * Autonomous State Machine
     */
    public void updateAutoRoutine(){
        SmartDashboard.putNumber("Current Auto Routine", currentAutonState);
        switch(currentAutonState){
            case START:
                stateActionStart();
                break;
            case MOVE_DISTANCE:
                stateActionInitMoveDistance();
                break;
            case TURN_LEFT:
                stateActionInitRocketTurn();
                break;
            case TURN_RIGHT:
                stateActionInitRocketTurn();
                break;
            case MOVE_VISION_TARGET:
                stateActionMoveVisionTarget();
                break;
            case AUTO_TURN_DELAY:
                stateAutoTurnDelay();
                break;
            case MOVE_DISTANCE_TARGET:
                stateActionRocketDeadReckoningMovement();
                break;
            case MOVE_DISTANCE_ROCKET:
                stateActionRocketMoveDistance();
                break;
            case RELEASE_HATCH:
                stateActionReleaseHatch();
                break;
            case BACK_TARGET:
                stateActionBackFromRocket();
                break;
            case STOP:
            default:
                stateActionStop();
            }
    }

    //called at the beginning of every autonomous method
    private void stateActionStart(){
        if(selectedAutonRoutine != DO_NOTHING){
        //go straight to stop if the selected auton routine was do nothing
        } else{
            currentAutonState = STOP;
        }
    }

    //handles init robot movement off of hab
    private void stateActionInitMoveDistance(){
        //wait for the robot to finish moving, and then go to the next state
        if(!DriveTrain.isMovingDistance()) {

        } else{
            //Wait for move to complete

        }
    }

    //handles turning in the direction of the rocket
    private void stateActionInitRocketTurn() {
        //when finished turning, continue to next state
        if(!DriveTrain.isTurning()) {

        } else {
            //wait for turn to complete

        }
    }

    //handles movement toward the rocket if dead reckoning was triggered
    private void stateActionRocketDeadReckoningMovement() {
        if (!DriveTrain.isMovingDistance()) {
            if (!isWithinTargetRange) {
                currentAutonState = STOP;

            } else {
                //TODO - handle autonomously placing hatch panels
                //currentAutonState = RELEASE_HATCH;
                currentAutonState = STOP;

            }

        } else {
            //wait to finish move distance
        }
    }

    //back away from the rocket
    private void stateActionBackFromRocket() {
        if(!DriveTrain.isMovingDistance()) {
            currentAutonState = STOP;

        } else {
            //wait to finish backing up

        }
    }

    //autonomously releases the hatch and begins to back away
    private void stateActionReleaseHatch() {
        //Intake.setHatchState(true);
        DriveTrain.setMoveDistance(Constants.kBackupDistance, -Constants.kBackupPower);
        currentAutonState = BACK_TARGET;

    }

    private void stateActionMoveVisionTarget() {
        
    }

    //after turn, move forward a little bit to straighten the robot out
    private void stateActionRocketMoveDistance() {
        if (!DriveTrain.isMovingDistance()) {
            setAutonDelay(0.5);
            currentAutonState = AUTO_TURN_DELAY;

        } else {
            //wait to finish move distance
        }
    }

    //once we wait a small amount of time, continue
    private void stateAutoTurnDelay() {
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
