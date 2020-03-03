/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightBase;


/**
 * A command that does nothing but takes a specified amount of time to finish.  Useful for
 * CommandGroups.  Can also be subclassed to make a command with an internal timer.
 */
public class trackToPortLeftAuto extends CommandBase {
  private final LimelightBase limelight;
  double desiredArea, acc, prevError;

  /**
   * Creates a new WaitCommand.  This command will do nothing, and end after the specified duration.
   *
   */
  public trackToPortLeftAuto() {
    limelight = Robot.m_robotContainer.tracker.limelight;
    acc = 0;
    prevError = 0;
    desiredArea = .342; 
  }

  @Override
  public void initialize() {
  }

  public double getY() {
    double kp = 3;

    double currentArea = limelight.get("ta");
    System.out.println(limelight.getTracking());
    System.out.println(limelight.get("ta"));
    double error = desiredArea - currentArea; //.015, .000005
    acc += error;
    double output = kp * error;// - .000005*(error - prevError)/.02;// + acc * .000005;
    prevError = error;
    return limelight.getTracking() ? output : .5;
  }

  public double getX() {
    double kp = .038;
    double error = limelight.get("tx");
    double output = kp * error;
    return limelight.getTracking() ? output : 0;
  }

  @Override
  public void execute() {
    Robot.m_robotContainer.m_drivebase.m_drive.arcadeDrive(getY(), getX());
  }
}
