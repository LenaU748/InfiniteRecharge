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

public class turn90From0 extends CommandBase {
  /**
   * Creates a new turnFromTrenc hToTargetLeftAuto.
   */
  double desiredAngle = 0;
  Timer turnTimer = new Timer();
  public turn90From0() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = (Robot.m_robotContainer.m_drivebase.m_gyro.getAngle() + 90);

    double error = Math.abs(desiredAngle - currentAngle);
    
    double kP = 0.0075;
    double output = (kP * error);
    
    Robot.m_robotContainer.m_drivebase.m_drive.arcadeDrive(0, output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnTimer.get() > 10; 
  }
}
