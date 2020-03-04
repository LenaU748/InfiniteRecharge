/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretBase extends SubsystemBase {

  public final CANSparkMax turretMotor = new CANSparkMax(Constants.TURR, MotorType.kBrushless);
  public final CANPIDController turretPID = turretMotor.getPIDController();
  public DigitalInput hallEffect = new DigitalInput(Constants.HALL);

  public TurretBase() {
    turretPID.setP(Constants.kTurretP);
    turretPID.setI(Constants.kTurretI);
    turretPID.setD(Constants.kTurretD);
    turretPID.setFF(Constants.kTurretFF);
    turretPID.setIZone(Constants.kTurretIZ);
    turretPID.setOutputRange(-0.25, 0.25);

    turretMotor.getEncoder().setPositionConversionFactor((1 / 46.67) * 360); // 46.67 motor rotations is one turret
                                                                             // rotation
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, 90);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -90);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPos(double setpoint) {
    turretPID.setReference(-setpoint, ControlType.kPosition);
  }
}
