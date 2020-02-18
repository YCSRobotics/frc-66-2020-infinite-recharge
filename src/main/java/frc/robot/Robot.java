/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default - Manual";
  private static final String kCenterFrontBayRight = "Center - Ship Right";
  private static final String kCenterFrontBayLeft = "Center - Ship Left";
  private static final String kRgtRocketLvl1 = "Right Rocket Level 1";
  private static final String kLftRocketLvl1 = "Left Rocket Level 1";
  
  private AutoRoutine autonomous = new AutoRoutine();
  private DriveTrain driveTrain = new DriveTrain();
  private Dashboard dashboard = new Dashboard();
  private Shooter shooter = new Shooter();
  //private FourBarControl fourBarControl = new FourBarControl();
  //private Intake intake = new Intake();
  //private ElevatorControl elevatorControl = new ElevatorControl();
  private Camera camera = new Camera();
  ///private Climber climber = new Climber();

  private String m_autonSelected = kDefaultAuto;
  private SendableChooser<String> m_chooser = new SendableChooser<>();

  //called on robot boot
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default - Manual", kDefaultAuto);
    m_chooser.setName("Sandstorm Auto");
    m_chooser.addOption("Center - Ship Right", kCenterFrontBayRight);
    m_chooser.addOption("Center - Ship Left", kCenterFrontBayLeft);
    m_chooser.addOption("Left Rocket Lvl 1", kLftRocketLvl1);
    m_chooser.addOption("Right Rocket Lvl 1", kRgtRocketLvl1);

    Dashboard.driverDisplayTab.add(m_chooser).withSize(2,1).withPosition(0,0);
  }

  //called every 20ms regardless of game state, after robot init
  @Override
  public void robotPeriodic() {
    dashboard.updateDiagDashboard();

  }

  //called at the beginning of auton
  @Override
  public void autonomousInit() {
    m_autonSelected = m_chooser.getSelected();

    System.out.println("Auto selected: " + m_autonSelected);

    DriveTrain.autonomousActive = true;

    switch(m_autonSelected){
      case kCenterFrontBayRight:
        autonomous.setSelectedAutonRoutine(AutoRoutine.CENTER_RIGHT);
        break;
      case kCenterFrontBayLeft:
        autonomous.setSelectedAutonRoutine(AutoRoutine.CENTER_LEFT);
        break;
      case kLftRocketLvl1:
        autonomous.setSelectedAutonRoutine(AutoRoutine.LEFT_ROCKET);
        break;
      case kRgtRocketLvl1:
        autonomous.setSelectedAutonRoutine(AutoRoutine.RIGHT_ROCKET);
        break;
      case kDefaultAuto:
      default:
              autonomous.setSelectedAutonRoutine(AutoRoutine.DO_NOTHING);
      break;
    }

  }

  //called every 20ms during auton, after auto init
  @Override
  public void autonomousPeriodic() {
    autonomous.updateAutoRoutine();
    driveTrain.updateDrivetrain();
    //fourBarControl.updateFourBar();
    //elevatorControl.updateLift();
    //intake.updateIntake();

  }

  @Override
  public void teleopInit() {
    DriveTrain.autonomousActive = false;
  }

  //called every 20ms during teleop
  @Override
  public void teleopPeriodic() {
    driveTrain.updateDrivetrain();
    shooter.updateShooter();
    
    //fourBarControl.updateFourBar();
    //elevatorControl.updateLift();
    //intake.updateIntake();
    //climber.updateClimber();

  }

  //called every 20ms during test mode
  @Override
  public void testPeriodic() {

  }
}
