/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class launchAuto extends CommandBase {
  /**
   * Creates a new launchAuto.
   */

  private Timer autoLaunchTimer = new Timer();
  public launchAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoLaunchTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_robotContainer.tracker.set(true);
    Robot.m_robotContainer.launchCommand.set(true, false, false, false, false);
    if (autoLaunchTimer.get() > 1) {
      Robot.m_robotContainer.launchCommand.feeder.feederMotor.set(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_robotContainer.tracker.set(false);
    Robot.m_robotContainer.launchCommand.set(false, false, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return autoLaunchTimer.get() > 4;
  }
}
