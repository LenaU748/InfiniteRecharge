/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ToggledSolenoid extends SubsystemBase {
  public DoubleSolenoid piston;
  public boolean toggleBool = false;
  boolean button;

  public ToggledSolenoid(int forward, int reverse) {
    piston = new DoubleSolenoid(forward, reverse);
  }

  public void togglePiston() {
    toggleBool = !toggleBool;
    piston.set(toggleBool ? Value.kForward : Value.kReverse);
  }

  public void set(boolean state) {
    piston.set(state ? Value.kForward : Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
