/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberBase extends SubsystemBase {
  /**
   * Creates a new ClimberBase.
   */
  public final CANSparkMax climberMotor = new CANSparkMax(Constants.CLIMBER_MOT, MotorType.kBrushless);
  public final ToggledSolenoid climberBrake = new ToggledSolenoid(Constants.CLIMBER_SOL, Constants.CLIMBER_SOL2);
  public final CANPIDController climberPID = climberMotor.getPIDController();

  public ClimberBase() {
    climberPID.setOutputRange(-1.0, 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
