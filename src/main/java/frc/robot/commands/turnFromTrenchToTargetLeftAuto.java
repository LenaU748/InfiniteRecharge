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

public class turnFromTrenchToTargetLeftAuto extends CommandBase {
  /**
   * Creates a new turnFromTrenc hToTargetLeftAuto.
   */
  double xinit, yinit, desiredAngle, acc, prevError;
  Timer turnTimer = new Timer();
  public turnFromTrenchToTargetLeftAuto() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnTimer.start();
    xinit = Robot.m_robotContainer.m_drivebase.m_odometry.getPoseMeters().getTranslation().getX();
    yinit = Robot.m_robotContainer.m_drivebase.m_odometry.getPoseMeters().getTranslation().getY();
    System.out.println(xinit);
    System.out.println(yinit);
    // desiredAngle = 175;//-Math.toDegrees(Math.atan2(-2.455-yinit, .295-xinit));
    // acc = 0;
    // prevError = 0;

    desiredAngle = 180;//Robot.m_robotContainer.m_drivebase.m_gyro.getYaw() + 180;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double theta = Robot.m_robotContainer.m_drivebase.initTheta();
    double currentAngle = (Robot.m_robotContainer.m_drivebase.m_gyro.getAngle() + theta)  % 360;
    double error = Math.abs(desiredAngle - currentAngle);
    
    double kP = 0.0075;
    //double kFF = 0.0035;
    double output = (kP * error);// + kFF;

    Robot.m_robotContainer.m_drivebase.m_drive.arcadeDrive(0, output);

    //////////////////////////////////////////////

    // System.out.println(desiredAngle);    
    // double angle = -Robot.m_robotContainer.m_drivebase.getHeading();
    // if (angle < 0) {
    //   angle += 360;
    // }
    // System.out.println(angle);    
    // double error = desiredAngle - angle; //.015, .000005
    // acc += error;
    // double output = .0075 * error;//  + acc * .000005;;// + .01*(error - prevError)/.02
    // Robot.m_robotContainer.m_drivebase.m_drive.arcadeDrive(0, output);
    // prevError = error;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnTimer.get() > 3; 
  }
}
