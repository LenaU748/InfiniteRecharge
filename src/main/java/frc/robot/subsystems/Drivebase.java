/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Drivebase extends SubsystemBase {

  // public final ToggledSolenoid shifterPistons = new
  // ToggledSolenoid(Constants.DRIVE_SOL1, Constants.DRIVE_SOL2);

  public final ToggledSolenoid shifterPistons = new ToggledSolenoid(Constants.DRIVE_SOL1, Constants.DRIVE_SOL2);

  public final CANSparkMax lfMotor = new CANSparkMax(Constants.LF, MotorType.kBrushless);
  public final CANSparkMax lbMotor = new CANSparkMax(Constants.LB, MotorType.kBrushless);
  public final CANSparkMax rfMotor = new CANSparkMax(Constants.RF, MotorType.kBrushless);
  public final CANSparkMax rbMotor = new CANSparkMax(Constants.RB, MotorType.kBrushless);

  public final Encoder leftEnc = new Encoder(4, 5);
  public final Encoder rightEnc = new Encoder(2, 3);

  public final SpeedControllerGroup leftMotors = new SpeedControllerGroup(lfMotor, lbMotor);
  public final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rfMotor, rbMotor);

  public DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  public AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  public double left_encPos;
  public double right_encPos;

  public double left_encVel;
  public double right_encVel;

  public String path = Robot.autoChooser.getSelected() == null ? "Nope" : Robot.autoChooser.getSelected();

  // public final SpeedControllerGroup leftMotors = new
  // SpeedControllerGroup(lfMotor, lbMotor);
  // public final SpeedControllerGroup rightMotors = new
  // SpeedControllerGroup(rfMotor, rbMotor);

  // public double left_encPos = -lfMotor.getSelectedSensorPosition() *
  // Constants.kEncoderDistancePerPulse;
  // public double right_encPos = rfMotor.getSelectedSensorPosition() *
  // Constants.kEncoderDistancePerPulse;

  // public double left_encVel = -lfMotor.getSelectedSensorVelocity() *
  // Constants.kEncoderDistancePerPulse;
  // public double right_encVel = rfMotor.getSelectedSensorVelocity() *
  // Constants.kEncoderDistancePerPulse;

  public final DifferentialDriveOdometry m_odometry;

  public Drivebase() {

    leftEnc.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    rightEnc.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    lfMotor.setIdleMode(IdleMode.kBrake);
    lbMotor.setIdleMode(IdleMode.kBrake);
    rfMotor.setIdleMode(IdleMode.kBrake);
    rbMotor.setIdleMode(IdleMode.kBrake);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
        new Pose2d(initX(), initY(), new Rotation2d(initTheta())));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    left_encPos = leftEnc.getDistance();
    right_encPos = rightEnc.getDistance();

    left_encVel = leftEnc.getRate();
    right_encVel = rightEnc.getRate();

    lbMotor.follow(lfMotor);
    rbMotor.follow(rfMotor);

    m_odometry.update(Rotation2d.fromDegrees(getHeading()), left_encPos, right_encPos);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left_encVel, right_encVel);
  }

  public void voltageControl(double leftVolts, double rightVolts) {
    lfMotor.setVoltage(leftVolts);
    rfMotor.setVoltage(-rightVolts);
    m_drive.feedWatchdog();
  }

  public double initX() {
    switch (path) {
    default:
    case "Nope":
      return 0;
    case "6 Ball Path":
      return 3.2;
    case "Left Turn from Init":
      return 3.1;
    case "Turn Line":
      return 1;
    case "3 Ball Forward":
      return 1;
    case "Line":
      return 1;
    }

  }

  public double initY() {
    switch (path) {
    default:
    case "Nope":
      return 0;
    case "6 Ball Path":
      return -0.7;
    case "Left Turn from Init":
      return -2.3;
    case "Turn Line":
      return -1;
    case "3 Ball Forward":
      return -1;
    case "Line":
      return -1;
    }
  }

  public double initTheta() {
    switch (path) {
    default:
    case "Nope":
      return Rotation2d.fromDegrees(0).getRadians();
    case "6 Ball Path":
      return Rotation2d.fromDegrees(-90).getRadians(); // -90 or 270?
    case "Left Turn from Init":
      return Rotation2d.fromDegrees(90).getRadians();
    case "Turn Line":
      return Rotation2d.fromDegrees(180).getRadians();
    case "3 Ball Forward":
      return Rotation2d.fromDegrees(0).getRadians();
    case "Line":
      return 0;
    }
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEnc.reset();
    rightEnc.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */

  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return -m_gyro.getYaw() + Math.toDegrees(initTheta());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

}
