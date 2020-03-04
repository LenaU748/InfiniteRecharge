/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autoCommand;
  public static SendableChooser<String> autoChooser = new SendableChooser<String>();
  public static RobotContainer m_robotContainer;
  public static Timer timer;
  // public int setpoint = 0;
  Compressor c = new Compressor();
  double prevTime;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Auto Chooser
    autoChooser.addOption("3 Ball Forward", "3 Ball Forward");
    autoChooser.addOption("6 Ball Path", "6 Ball Path");
    autoChooser.addOption("Left Turn", "Left Turn");
    autoChooser.addOption("Line", "Line");

    SmartDashboard.putData("Autonomous routine", autoChooser);
    m_robotContainer.roboInit();

    // LIDAR
    m_robotContainer.launchCommand.launcher.lidar.startMeasuring();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
    m_robotContainer.roboPeriodic();
    m_robotContainer.launchCommand.schedule();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  @Override
  public void autonomousInit() {
    m_robotContainer.autoInit();
    m_robotContainer.m_drivebase.m_gyro.reset();
    m_robotContainer.m_drivebase.resetOdometry(
        new Pose2d(new Translation2d(m_robotContainer.m_drivebase.initX(), m_robotContainer.m_drivebase.initY()),
            new Rotation2d(m_robotContainer.m_drivebase.initTheta())));
    if (autoChooser.getSelected() != null) {
      m_autoCommand = m_robotContainer.getAutonomousCommand(autoChooser.getSelected());
      m_autoCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // m_robotContainer.m_driveAuto.m_drive.tankDrive(0.8, 0.8);
    // m_robotContainer.m_driveAuto.m_drive.feed();
    m_robotContainer.tracker.schedule();
  }

  @Override
  public void teleopInit() {
    m_robotContainer.m_drivebase.resetOdometry(
        new Pose2d(new Translation2d(m_robotContainer.m_drivebase.initX(), m_robotContainer.m_drivebase.initY()),
            new Rotation2d(m_robotContainer.m_drivebase.initTheta())));
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }

    m_robotContainer.teleopIn();
    c.setClosedLoopControl(true);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_robotContainer.launcherTimer.reset();
    m_robotContainer.launcherTimer.stop();
    m_robotContainer.feedReady = false;
    m_robotContainer.indexReady = false;
    m_robotContainer.shootReady = false;
    m_robotContainer.launchCommand.feeder.ballCount = 0; // m_robotContainer.m_driveAuto.resetEncoders();
    // m_robotContainer.m_turretBase.zeroTurret();
    // m_robotContainer.m_driveAuto.m_gyro.reset();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleop();

    SmartDashboard.putNumber("Ball Count", m_robotContainer.launchCommand.feeder.ballCount);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}