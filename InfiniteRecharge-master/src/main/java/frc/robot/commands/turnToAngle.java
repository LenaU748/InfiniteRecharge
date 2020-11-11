/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class turnToAngle extends CommandBase {
  /**
   * Creates a new turnFromTrenc hToTargetLeftAuto.
   */
  double desiredAngle;
  Timer turnTimer = new Timer();
  public turnToAngle(double DA) {
    // Use addRequirements() here to declare subsystem dependencies.
    desiredAngle = DA;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double theta = Robot.m_robotContainer.m_drivebase.initAngle();
    double currentAngle = (Robot.m_robotContainer.m_drivebase.m_gyro.getAngle() + theta);

    double error = Math.abs(desiredAngle - currentAngle);
    
    double kP = 0.0075;
    double output = (kP * error);

    SmartDashboard.putNumber("Current Angle", currentAngle);
    SmartDashboard.putNumber("Gyro Angle", Robot.m_robotContainer.m_drivebase.m_gyro.getAngle());
    SmartDashboard.putNumber("Initial Angle", theta);
    SmartDashboard.putNumber("Angle Error", error);
    SmartDashboard.putNumber("Angle Output", output);
    
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
