/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.FeederBase;
import frc.robot.subsystems.LauncherBase;

public class launch extends CommandBase {
  /**
   * Creates a new launch.
   */
  boolean shootReady;
  public LauncherBase launcher = new LauncherBase();
  public FeederBase feeder = new FeederBase();
  double distance;
  boolean shooting;
  boolean indexing;
  boolean trenchShot;
  boolean indexOut;
  double trenchRPM = 5600;

  public launch(boolean _shootReady) {
    // Use addRequirements() here to declare subsystem dependencies.
    shootReady = _shootReady;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public void set(boolean state, boolean _shooting, boolean _indexing, boolean _trenchShot, boolean _indexOut) {
    shootReady = state;
    shooting = _shooting;
    indexing = _indexing;
    trenchShot = _trenchShot;
    indexOut = _indexOut;
  }

  public boolean inRange() {
    System.out.println(launcher.lLaunchMotor.getEncoder().getVelocity());
    // May need to fix this range
    if (shootReady || trenchShot) {
      if (shootReady) {
        return (launcher.lLaunchMotor.getEncoder().getVelocity() >= .98 * launcher.calculateRPMModel()
            && launcher.lLaunchMotor.getEncoder().getVelocity() < (1.02 * launcher.calculateRPMModel()))
            || launcher.lLaunchMotor.getEncoder().getVelocity() > 5800;
      } else {
        return (launcher.lLaunchMotor.getEncoder().getVelocity() >= .98 * trenchRPM
            && launcher.lLaunchMotor.getEncoder().getVelocity() < (1.02 * trenchRPM))
            || launcher.lLaunchMotor.getEncoder().getVelocity() > 5800;
      }
    } else {
      return true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (indexOut) {
      feeder.indexMotor.set(-0.75);
    } else if (feeder.ballCount < Constants.maxBallCount) {
      feeder.indexMotor.set(.75);
    } else {
      feeder.indexMotor.set(0);
    }

    if (shootReady || trenchShot) {

      if (shootReady) {
        launcher.setRPM(launcher.calculateRPMModel());
      } else {
        launcher.setRPM(trenchRPM);
      }

      if (inRange()) {
        feeder.feederMotor.set(0.5);
        feeder.ballCount = 0;
      } else {
        feeder.feederMotor.set(0);
      }
    } else {
      if (indexing) {

        if (feeder.limitSwitchPressed()) {
          feeder.feederPID.setReference(feeder.feederPos + Constants.feederRevForBall, ControlType.kPosition);
          feeder.ballCount++;
        }

      } else {
        feeder.feederMotor.set(0);
      }

      if (shooting) {
        launcher.setRPM(5300);
      } else {
        launcher.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.m_robotContainer.opJoy.getTriggerAxis(Hand.kLeft) > 0.05
        || Robot.m_robotContainer.opJoy.getTriggerAxis(Hand.kRight) > 0.05
        || Robot.m_robotContainer.driveJoy.getBackButton();
  }
}
