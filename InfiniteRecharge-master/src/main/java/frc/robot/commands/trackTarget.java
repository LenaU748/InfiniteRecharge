/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LimelightBase;
import frc.robot.subsystems.TurretBase;

public class trackTarget extends CommandBase {
  /**
   * Creates a new trackTarget.
   */

  public LimelightBase limelight = new LimelightBase();
  public TurretBase turret = new TurretBase();
  boolean tracking = false;

  public trackTarget(boolean _tracking) {
    // Use addRequirements() here to declare subsystem dependencies.
    tracking = _tracking;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelight.setTracking(tracking);
    if (tracking) {
      Robot.m_robotContainer.setpoint = -turret.turretMotor.getEncoder().getPosition()
          + (limelight.get("tx") + (1.25 / 154) * Robot.m_robotContainer.launchCommand.launcher.lidar.getDistance());
      turret.turretPID.setI(0.000025);
    }
    turret.turretPID.setI(0.00000);
    turret.setPos(Robot.m_robotContainer.setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  public void set (boolean bool) {
    tracking = bool;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
