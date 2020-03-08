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
  private static final String kDefaultAuto = "Default - None";
  private static final String kMoveOnly = "Move Only";
  private static final String kShootAutoLine = "Auto Line Shoot";
  private static final String kKesselRun = "Kessel Run";
  
  private AutoRoutine autonomous = new AutoRoutine();
  private DriveTrain driveTrain = new DriveTrain();
  private Dashboard dashboard = new Dashboard();
  private Shooter shooter = new Shooter();
  private Intake intake = new Intake();

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
    m_chooser.setDefaultOption("Default - None", kDefaultAuto);
    m_chooser.addOption("Move Only", kMoveOnly);
    m_chooser.addOption("Auto Line Shoot", kShootAutoLine);
    m_chooser.addOption("Kessel Run", kKesselRun);

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
    camera.initCamera();
    autonomous.initAutoRoutine();

    m_autonSelected = m_chooser.getSelected();

    System.out.println("Auto selected: " + m_autonSelected);

    DriveTrain.autonomousActive = true;

    switch(m_autonSelected){

      case kMoveOnly:
        autonomous.setSelectedAutonRoutine(AutoRoutine.MOVE_ONLY);
        break;
      case kShootAutoLine:
        autonomous.setSelectedAutonRoutine(AutoRoutine.AUTO_LINE_SHOOT);
        break;
      case kKesselRun:
        autonomous.setSelectedAutonRoutine(AutoRoutine.KESSEL_RUN);
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
    camera.updateCamera();
    driveTrain.updateDrivetrainAuto();
    shooter.updateShooter();
  }

  @Override
  public void teleopInit() {
    DriveTrain.autonomousActive = false;
    camera.initCamera();
    intake.initIntake();
  }

  //called every 20ms during teleop
  @Override
  public void teleopPeriodic() {
    camera.updateCamera();
    driveTrain.updateDrivetrain();
    shooter.updateShooter();
    intake.updateIntake();
  }

  //called every 20ms during test mode
  @Override
  public void testPeriodic() {

  }
}
