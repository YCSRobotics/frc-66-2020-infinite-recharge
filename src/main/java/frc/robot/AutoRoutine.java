package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoRoutine {
    //Timer for timed delays
    private Timer timer = new Timer();

    //Autonomous Routines
    final static int DO_NOTHING           = 0;
    final static int CENTER_LEFT          = 1;
    final static int CENTER_RIGHT         = 2;
    final static int LEFT_ROCKET          = 3;
    final static int RIGHT_ROCKET         = 4;

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
            //if center, move distance and set state to move distance
            if((selectedAutonRoutine == CENTER_LEFT) || (selectedAutonRoutine == CENTER_RIGHT)) {
                DriveTrain.setMoveDistance(Constants.kCenterGoStraightInitDistance, Constants.kCenterGoStraightInitPower);
                currentAutonState = MOVE_DISTANCE;
            
            //if rocket, move rocket distance
            } else if ((selectedAutonRoutine==LEFT_ROCKET) || (selectedAutonRoutine == RIGHT_ROCKET)) {
                DriveTrain.setMoveDistance(Constants.kRocketInitDistance, Constants.kRocketInitPower);
                currentAutonState = MOVE_DISTANCE;

            } else {
                //should never get here as this would mean we didn't account for a selected autonomous
                currentAutonState = STOP;
            }

        //go straight to stop if the selected auton routine was do nothing
        } else{
            currentAutonState = STOP;
        }
    }

    //handles init robot movement off of hab
    private void stateActionInitMoveDistance(){
        //wait for the robot to finish moving, and then go to the next state
        if(!DriveTrain.isMovingDistance()) {
            //if center selected, move forward off of vision if target detected, else just stop
            if((selectedAutonRoutine == CENTER_LEFT) || (selectedAutonRoutine == CENTER_RIGHT)) {
                if (SensorData.tapeDetected()) {
                    DriveTrain.moveToVisionTarget(Constants.kVisionPower);
                    currentAutonState = MOVE_VISION_TARGET;

                } else {
                    currentAutonState = STOP;

                }

            //if rocket selected, begin to turn
            } else if (selectedAutonRoutine == RIGHT_ROCKET) {
                DriveTrain.setTurnToTarget(Constants.kRocketTurnPower, Constants.kRocketTurnAngle);
                currentAutonState = TURN_RIGHT;

            } else if(selectedAutonRoutine == LEFT_ROCKET) {
                DriveTrain.setTurnToTarget(-Constants.kRocketTurnPower, Constants.kRocketTurnAngle);
                currentAutonState = TURN_LEFT;

            } else {
                currentAutonState = STOP;
            }

        } else{
            //Wait for move to complete

        }
    }

    //handles turning in the direction of the rocket
    private void stateActionInitRocketTurn() {
        //when finished turning, continue to next state
        if(!DriveTrain.isTurning()) {
            if (selectedAutonRoutine == LEFT_ROCKET || selectedAutonRoutine == RIGHT_ROCKET) {
                DriveTrain.setMoveDistance(Constants.kRocketSecondDistance, Constants.kRocketSecondPower);
                currentAutonState = MOVE_DISTANCE_ROCKET;

            } else {
                currentAutonState = STOP;

            }

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
        Intake.setHatchState(true);
        DriveTrain.setMoveDistance(Constants.kBackupDistance, -Constants.kBackupPower);
        currentAutonState = BACK_TARGET;

    }

    private void stateActionMoveVisionTarget() {
        //if center selected, and no longer following target, stop
        if (selectedAutonRoutine == CENTER_LEFT || selectedAutonRoutine == CENTER_RIGHT) {
            if (!DriveTrain.isFollowingTarget()) {
                currentAutonState = STOP;
            }

        //if rocket selected, and no longer following target, move forward ramming distance
        } else if (selectedAutonRoutine == LEFT_ROCKET || selectedAutonRoutine == RIGHT_ROCKET) {
            if (!DriveTrain.isFollowingTarget()) {
                if (isWithinTargetRange) {
                    //ram that BOI
                    DriveTrain.setMoveDistance(Constants.kRammingDistance, Constants.kRammingPower);
                    currentAutonState = MOVE_DISTANCE_TARGET;

                } else {
                    //not within range, lost target, stop
                    currentAutonState = STOP;

                }

            } else {
                //Waiting to finish trackingTarget

            }
        }
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
        if(timer.get() >= alarmTime) {
            //if rocket and target found, track using vision, otherwise use dead reckoning state
            if((selectedAutonRoutine == LEFT_ROCKET) || (selectedAutonRoutine == RIGHT_ROCKET)){
                if (SensorData.tapeDetected()) {
                    DriveTrain.moveToVisionTarget(Constants.kVisionPower);
                    currentAutonState = MOVE_VISION_TARGET;

                } else {
                    System.out.println("Using deadreckoning rocket distance");
                    DriveTrain.setMoveDistance(Constants.kRocketDeadReckoningDistance, Constants.kRocketDeadReckoningPower);
                    currentAutonState = MOVE_DISTANCE_TARGET;

                }

            } else{
                //Should never get here, but if we do Stop
                currentAutonState = STOP;

            }

        } else{
            //Wait for timer to expire
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
