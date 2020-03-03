/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.IntakeBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class autoIntake extends CommandBase {
  /**
   * Creates a new autoIntake.
   */
  private IntakeBase intake;
  public autoIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = Robot.m_robotContainer.m_intakeBase;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.intakePistons.set(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeMotor.set(-.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakePistons.set(false);
    intake.intakeMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
