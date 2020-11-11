/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {

    // Max Current Limit
    public final static int LIM1 = 20;
    public final static int LIM2 = 30;

    // DRIVEBASE
    // Motor Controllers
    public final static int LF = 4;
    public final static int LB = 5;
    public final static int RF = 2;
    public final static int RB = 3;
    // Solenoids
    // Air brake: 0, 3
    // Shifter: 2, 5
    // Intake: 1, 4
    public final static int DRIVE_SOL1 = 2;
    public final static int DRIVE_SOL2 = 5;

    // FEEDER AND INDEXER
    // Motor Controllers
    public final static int FEED = 6;
    public final static int INDEX = 7;
    // PID
    public static double kFeederP = .5;
    public static double kFeederI = 0;
    public static double kFeederD = 0;
    public static double kFeederFF = 0;
    public static double kFeederIZ = 0;
    // Other
    public static double feederRevForBall = 16;
    public static double maxBallCount = 3;

    // TURRET AND LAUNCHER
    // Motor Controllers
    public final static int TURR = 8;
    public final static int RLAUNCH = 9;
    public final static int LLAUNCH = 10;

    // PID
    public final static double kLauncherP = 0;// .0000035;
    public final static double kLauncherI = 0.000000002;
    public final static double kLauncherD = 0;
    public final static double kLauncherFF = .00017647;
    public final static double kLauncherIZ = 0.0;

    public final static double kTurretP = 0.00725;
    public final static double kTurretI = 0.000025;
    public final static double kTurretD = 0.00;
    public final static double kTurretFF = 0.0;
    public final static double kTurretIZ = 0.0;

    // public final static double launcherSpeedupTime = 3;

    // INTAKE CONSTANTS
    // Motor Controllers
    public final static int INTAKE = 1;

    // Solenoids
    public final static int INTAKE_SOL1 = 1;
    public final static int INTAKE_SOL2 = 4;

    // SENSOR CONSTANTS
    // Sensors
    public final static int HALL = 0;

    // JOYSTICK CONSTANTS
    // Joystick Ports
    public static final int zero = 0;
    public static final int one = 1;

    // Joystick Axes
    public static final int XL = 0;
    public static final int YL = 1;
    public static final int XR = 4;
    public static final int YR = 5;

    // CLIMBER CONSTANTS
    // MOTORS
    public final static int CLIMBER_MOT = 11;

    // SOLENOIDS
    public final static int CLIMBER_SOL = 0;
    public final static int CLIMBER_SOL2 = 3;
    // PID
    public static final double kclimberP = 0; // all need to change!!!
    public static final double kclimberI = 0; // all need to change!!!
    public static final double kclimberD = 0; // all need to change!!!
    public static final double kclimberFF = 0; // all need to change!!!
    public static final double kclimberIZ = 0; // all need to change!!!

    // Characterization Toolsuite Constants
    public static final double ksVolts = 0.205, 
                               kvVoltSecondsPerMeter = 2.1, 
                               kaVoltSecondsSquaredPerMeter = 0.217,
                               kTrackwidthMeters = 0.653, 
                               kP = 5, // 8.87,
                               kD = 0.0, 
                               kMaxSpeedMetersPerSecond = 4.953, // 4,//4.953,//2.5 // Need to be changed
                               kMaxAccelerationMetersPerSecondSquared = 8.5344, // 4,//6,//8.5344,//4 // Need to be changed
                               kRamseteB = 2, 
                               kRamseteZeta = 0.7;

    public static final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // Encoder Constants
    public static final double kEncoderDistancePerPulse = (6 * Math.PI * 2.54 / 100) / (2048);
    // ~= 0.001329

    public static final boolean kGyroReversed = false;

    // CPM stuffs
    public static final I2C.Port kColorSensor = I2C.Port.kOnboard;
    public static final int kCPMotor = 4;
    public static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
    public static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);

    public static HashMap<String, Color> FMStoColor = new HashMap<String, Color>();

    public Constants() {
        FMStoColor.put("R", kBlueTarget);
        FMStoColor.put("G", kYellowTarget);
        FMStoColor.put("B", kRedTarget);
        FMStoColor.put("Y", kGreenTarget);
    }
}
