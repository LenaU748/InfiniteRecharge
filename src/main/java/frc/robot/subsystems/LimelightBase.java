/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.networktables.NetworkTable;

public class LimelightBase extends SubsystemBase {
  NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  double tv = get("tv");

  public LimelightBase() {
    m_limelightTable.getEntry("pipeline").setNumber(0);
  }

  public double get(String var) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(var).getDouble(0.0);
  }

  public void setTracking(boolean tracking) {
    m_limelightTable.getEntry("camMode").setNumber(tracking ? 0 : 1);
    m_limelightTable.getEntry("ledMode").setNumber(tracking ? 0 : 1);
  }

  public boolean getTracking() {
    return get("tv") == 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
