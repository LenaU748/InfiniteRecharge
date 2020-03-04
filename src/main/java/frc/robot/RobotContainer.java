/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.turnToAngle;
import frc.robot.commands.autoIntake;
import frc.robot.commands.launch;
import frc.robot.commands.launchAuto;
import frc.robot.commands.turn90From0;
import frc.robot.commands.trackTarget;
import frc.robot.commands.trackToPortLeftAuto;
import frc.robot.commands.turn180From0;
import frc.robot.subsystems.ClimberBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.IntakeBase;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // public double setpoint = 0;
  public double setpoint;

  public final Drivebase m_drivebase = new Drivebase();
  public final IntakeBase m_intakeBase = new IntakeBase();
  public final ClimberBase m_climberBase = new ClimberBase();
  public boolean indexOut = false;
  public final launch launchCommand = new launch(false);
  public final trackTarget tracker = new trackTarget(false);

  // Control Scheme
  public static final double JOY_DEADZONE = 0.2;
  boolean quickTurn = false;
  boolean feedReady = false;
  boolean indexReady = false;
  boolean shootReady = false;
  boolean shiftState = false; // check this
  Timer launcherTimer = new Timer();
  double smolrecord = 100000;
  Trajectory trajectory;
  public double rpmSet;
  boolean prevLAxis = false;
  boolean prevRAxis = false;
  boolean prevRBumper = false;

  // Initialize joysticks
  public final XboxController driveJoy = new XboxController(1);
  public final XboxController opJoy = new XboxController(0);

  // Buttons
  public POVButton up = new POVButton(opJoy, 0);
  public POVButton left = new POVButton(opJoy, 270);
  public POVButton right = new POVButton(opJoy, 90);
  public POVButton upright = new POVButton(opJoy, 45);
  public POVButton upleft = new POVButton(opJoy, 315);

  // Joystick Methods
  public double getDriveJoy(int axis) {
    double raw = driveJoy.getRawAxis(axis);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

  public double getOpJoy(int axis) {
    double raw = opJoy.getRawAxis(axis);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
  }

  public double getDriveJoyXR() {
    double raw = getDriveJoy(Constants.XR);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  public double getDriveJoyYL() {
    double raw = getDriveJoy(Constants.YL);
    return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw > 0 ? (raw * raw) / 1.5 : (-raw * raw) / 1.5;
  }

  // Match Period Code
  public void roboInit() {
  }

  public void roboPeriodic() {
    tracker.schedule();
  }

  public void autoInit() {
    setpoint = tracker.turret.turretMotor.getEncoder().getPosition(); // might have to add a negative
  }

  public void teleopIn() {
  }

  public void teleop() {
    // SMARTDASHBOARD

    // DRIVE JOYSTICK
    // // DRIVEBASE

    if (Math.abs(getDriveJoy(Constants.YL)) > 0.2) {
      m_drivebase.m_drive.curvatureDrive(-getDriveJoy(Constants.YL), getDriveJoyXR(), driveJoy.getBButtonPressed());
    } else {
      m_drivebase.m_drive.arcadeDrive(-getDriveJoy(Constants.YL), getDriveJoyXR());
    }

    if (driveJoy.getXButtonPressed()) {
      m_drivebase.shifterPistons.togglePiston();
      shiftState = true;
    }

    SmartDashboard.putString("Gear", shiftState ? "Low" : "High");

    // OPERATOR JOYSTICK
    // // INTAKE
    if (driveJoy.getTriggerAxis(Hand.kRight) > 0.05) {
      // intake balls
      m_intakeBase.intakeMotor.set(-driveJoy.getTriggerAxis(Hand.kRight));
    } else if (driveJoy.getTriggerAxis(Hand.kLeft) > 0.05) {
      // spit out balls
      m_intakeBase.intakeMotor.set(driveJoy.getTriggerAxis(Hand.kLeft));
    } else {
      m_intakeBase.intakeMotor.set(0);
    }

    if (driveJoy.getBumperPressed(Hand.kRight)) {
      m_intakeBase.intakePistons.togglePiston();
    }

    // LAUNCH SYSTEM
    launchCommand.feeder.update();
    if (opJoy.getBumper(Hand.kRight)) {
      shootReady = true;
      tracker.set(true);
    } else if (opJoy.getBButton()) {
      tracker.set(true);
    } else {
      shootReady = false;
      tracker.set(false);
    }

    boolean shooting;
    if (opJoy.getBumper(Hand.kLeft)) {
      shooting = true;
    } else {
      shooting = false;
    }

    boolean trenchShot;
    if (opJoy.getAButton()) {
      trenchShot = true;
    } else {
      trenchShot = false;
    }

    boolean indexing = launchCommand.feeder.ballCount <= Constants.maxBallCount;

    if (driveJoy.getStartButton()) {
      indexOut = true;
    } else {
      indexOut = false;
    }

    launchCommand.set(shootReady, shooting, indexing, trenchShot, indexOut);
    launchCommand.schedule();
    boolean RAxis = opJoy.getTriggerAxis(Hand.kRight) > 0.05;
    boolean LAxis = opJoy.getTriggerAxis(Hand.kLeft) > 0.05;
    boolean RBumper = opJoy.getBumper(Hand.kRight);

    if (opJoy.getBackButton()) {
      launchCommand.feeder.ballCount = 0;
    }

    if (LAxis) {
      launchCommand.feeder.feederMotor.set(0.5);
    } else if (RAxis) {
      launchCommand.feeder.feederMotor.set(-0.5);
    } else if ((!RAxis && prevRAxis) || (!LAxis && prevLAxis) || opJoy.getBumperReleased(Hand.kRight)
        || opJoy.getAButtonReleased()) {
      launchCommand.feeder.feederMotor.set(0);
    } else if (!shootReady && !shooting && !indexing) {
      launchCommand.feeder.feederMotor.set(0);
    }

    prevLAxis = LAxis;
    prevRAxis = RAxis;
    prevRBumper = RBumper;

    if (up.get()) {
      setpoint = 0;
    } else if (left.get()) {
      setpoint = -90;
    } else if (right.get()) {
      setpoint = 90;
    } else if (upright.get()) {
      setpoint = 45;
    } else if (upleft.get()) {
      setpoint = -45;
    }

    // CLIMBER
    if (opJoy.getXButton()) {
      m_climberBase.climberMotor.set(-1.0);
      m_climberBase.climberBrake.piston.set(Value.kReverse);
    } else if (opJoy.getYButton()) {
      m_climberBase.climberMotor.set(1.0);
      m_climberBase.climberBrake.piston.set(Value.kReverse);
    } else {
      m_climberBase.climberMotor.set(0);
      m_climberBase.climberBrake.piston.set(Value.kForward);
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String path) {
    switch (path) {
    case "6 Ball Path":
      return new launchAuto().andThen(new turnToAngle(90))
          .andThen(pathFollow("output/6 Ball Part 1.wpilib.json").alongWith(new autoIntake()));
    // return new launchAuto().andThen(pathFollow("output/6 Ball Path Part
    // 1.wpilib.json")).alongWith(new autoIntake()).andThen(new launchAuto());
    case "6 Ball Manual":
      return new launchAuto().andThen(new turn90From0())
          .andThen(pathFollow("output/6 Ball Part 1.wpilib.json").alongWith(new autoIntake()))
          .andThen(new turn180From0()).andThen(new trackToPortLeftAuto())
          .alongWith(new WaitCommand(2).andThen(new launchAuto()));
    case "Left Turn":
      return new turnToAngle(0);
    case "3 Ball Forward":
      return new launchAuto().andThen(pathFollow("output/Line.wpilib.json"));
    case "Line":
      return pathFollow("output/Line.wpilib.json");
    }
    return null;
  }

  public Command pathFollow(String pathLocation) {
    String trajectoryJSON = pathLocation;
    try {
      Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
    } catch (final IOException ex) {
      // TODO Auto-generated catch block
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    final RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivebase::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.m_driveKinematics, m_drivebase::getWheelSpeeds, new PIDController(Constants.kP, 0, 0),
        new PIDController(Constants.kP, 0, 0), m_drivebase::voltageControl, m_drivebase);
    // Run path following command, then stop at the end.
    // Robot.m_robotContainer.m_driveAuto.m_drive.feed();
    return ramseteCommand;
  }
}
