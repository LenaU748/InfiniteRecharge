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

public class IntakeBase extends SubsystemBase {
  public final ToggledSolenoid intakePistons = new ToggledSolenoid(Constants.INTAKE_SOL1, Constants.INTAKE_SOL2);
  public final CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
  public final CANPIDController intakePID = intakeMotor.getPIDController();

  public IntakeBase() {
    intakeMotor.setSmartCurrentLimit(Constants.LIM2);
    //configPeakCurrentLimit(Constants.LIM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
