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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherBase extends SubsystemBase {
  /**
   * Creates a new LauncherBase.
   */
  public final CANSparkMax lLaunchMotor = new CANSparkMax(Constants.LLAUNCH, MotorType.kBrushless);
  public final CANSparkMax rLaunchMotor = new CANSparkMax(Constants.RLAUNCH, MotorType.kBrushless);
  public final CANPIDController lLaunchPID = lLaunchMotor.getPIDController();
  public final CANPIDController rLaunchPID = rLaunchMotor.getPIDController();
  public final SpeedControllerGroup m_launchMotors = new SpeedControllerGroup(lLaunchMotor, rLaunchMotor);
  public final LidarBase lidar = new LidarBase(Port.kMXP);
  public double distance = lidar.getDistance();

  
  public LauncherBase() {
    lidar.startMeasuring();
    lLaunchMotor.setInverted(true);
    
    lLaunchPID.setP(Constants.kLauncherP);
    lLaunchPID.setI(Constants.kLauncherI);
    lLaunchPID.setD(Constants.kLauncherD);
    lLaunchPID.setFF(Constants.kLauncherFF);
    lLaunchPID.setIZone(Constants.kLauncherIZ);
    lLaunchPID.setOutputRange(-1.0, 1.0);

    rLaunchPID.setP(Constants.kLauncherP);
    rLaunchPID.setI(Constants.kLauncherI);
    rLaunchPID.setD(Constants.kLauncherD);
    rLaunchPID.setFF(Constants.kLauncherFF);
    rLaunchPID.setIZone(Constants.kLauncherIZ);
    rLaunchPID.setOutputRange(-1.0, 1.0);
  }

  public void setRPM(double rpm) {

    lLaunchPID.setReference(rpm, ControlType.kVelocity);
    rLaunchPID.setReference(rpm, ControlType.kVelocity);
  }

  public void stop() {
    lLaunchMotor.set(0);
    rLaunchMotor.set(0);
  }

  
  public double calculateRPMModel() {
    return (0.0745 * Math.pow(distance, 2)) + (-26.7 * distance) + 7333;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
