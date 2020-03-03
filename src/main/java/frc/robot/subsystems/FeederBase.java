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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederBase extends SubsystemBase {

  public final CANSparkMax feederMotor = new CANSparkMax(Constants.FEED, MotorType.kBrushless);
  public final CANSparkMax indexMotor = new CANSparkMax(Constants.INDEX, MotorType.kBrushless);
  public final CANPIDController feederPID = feederMotor.getPIDController();
  public final DigitalInput forwardLimitSwitch = new DigitalInput(1);
  public boolean prevLimitSwitch = false;
  public double feederPos = feederMotor.getEncoder().getPosition();
  public int ballCount = 0;

  public FeederBase() {
    feederMotor.setSmartCurrentLimit(Constants.LIM1);
    indexMotor.setSmartCurrentLimit(Constants.LIM1);
    feederMotor.getEncoder().setPositionConversionFactor(1);

    feederPID.setP(Constants.kFeederP);
    feederPID.setI(Constants.kFeederI);
    feederPID.setD(Constants.kFeederD);
    feederPID.setFF(Constants.kFeederFF);
    feederPID.setIZone(Constants.kFeederIZ);
    feederPID.setOutputRange(-.25, .25);
  }

  public boolean limitSwitchPressed() {
    if (!forwardLimitSwitch.get() && prevLimitSwitch) {
      prevLimitSwitch = forwardLimitSwitch.get();
      return true;
    } else {
      prevLimitSwitch = forwardLimitSwitch.get();
      return false;
    }
  }
  public void update() {
    feederPos = feederMotor.getEncoder().getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
