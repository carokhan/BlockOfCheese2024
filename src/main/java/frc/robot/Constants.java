// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode =
      RobotBase.isSimulation() ? Mode.SIM : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class RobotMap {
    public static class Drive {
      public static final int frontLeftDrive = 1;
      public static final int frontLeftTurn = 2;
      public static final int frontRightDrive = 3;
      public static final int frontRightTurn = 4;
      public static final int backLeftDrive = 5;
      public static final int backLeftTurn = 6;
      public static final int backRightDrive = 7;
      public static final int backRightTurn = 8;

      public static final boolean frontLeftTurnInvert = true;
      public static final boolean frontRightTurnInvert = true;
      public static final boolean backLeftTurnInvert = true;
      public static final boolean backRightTurnInvert = true;

      public static final int frontLeftEncoder = 0;
      public static final int frontRightEncoder = 1;
      public static final int backLeftEncoder = 2;
      public static final int backRightEncoder = 3;

      public static final double frontLeftOffset =
          Rotation2d.fromDegrees(-78.560751073938).getRadians(); // 101.8585654710
      public static final double frontRightOffset =
          Rotation2d.fromDegrees(-175.70581168291014).getRadians(); // 39.585409
      public static final double backLeftOffset =
          Rotation2d.fromDegrees(-73.54207322946).getRadians(); // 103.4415122
      public static final double backRightOffset =
          Rotation2d.fromDegrees(-72.38969990).getRadians(); // 110.898736

      public static final int gyro = 10;
    }
  }

  public static class AutoConstants {
    public static final double kPTranslation = 8.5;
    public static final double kDTranslation = 0.0;

    public static final double kPRotation = 3.0;
    public static final double kDRotation = 0.0;

    public static final double mass = DriveConstants.mass;
    public static final double moi = DriveConstants.angularMOI;
    public static final double bumperFront = Units.inchesToMeters(33.5 / 2);
    public static final double bumperBack = Units.inchesToMeters(33.5 / 2);
    public static final double bumperSide = Units.inchesToMeters(32.5 / 2);
    public static final double frontModX = DriveConstants.trackWidthY / 2;
    public static final double frontLeftY = DriveConstants.trackWidthX / 2;
    public static final double backModX = -DriveConstants.trackWidthY / 2;
    public static final double backLeftY = -DriveConstants.trackWidthX / 2;
    public static final double wheelRadius = DriveConstants.wheelRadius;
    public static final double motorRevWheelRev = DriveConstants.driveRatio;
    public static final double motorMaxSpeed =
        5880 * 0.8 * Math.PI * 2 / 60; // RPM (free speed) * 0.8 * 2pi/60 = radPerSec (loaded speed)
    public static final double motorMaxTorque =
        3.28 / 181 * DriveConstants.driveCurrent; // (kT * currentLimit)
  }

  public static class DriveConstants {
    public static final boolean wheelsStraight = false;

    public static final double trackWidthX = Units.inchesToMeters(19.75);
    public static final double trackWidthY = Units.inchesToMeters(20.75);
    public static final double trackBaseRadius = Math.hypot(trackWidthX / 2.0, trackWidthY / 2.0);

    public static final double wheelRadius = Units.inchesToMeters(2);

    public static final double mass = Units.lbsToKilograms(56);

    public static final double driveRatio = 5.36;
    public static final double driveMOI = 0.025;
    public static final double turnRatio = 150.0 / 7.0;
    public static final double turnMOI = 0.004;

    public static final double driveConversion = (driveRatio) * (1.0 / (wheelRadius * 2 * Math.PI));
    public static final double turnConversion = 2 * Math.PI / turnRatio;
    public static final double turnVelocityConversion = turnConversion / 60;

    public static final int driveCurrent = 40; // 70
    public static final int turnCurrent = 40; // 30

    public static final double odometeryFrequency = 250;
    public static final double updateFrequency = 100;

    public static final double maxLinearVelocity = Units.feetToMeters(20.4);
    // public static final double maxLinearVelocity = Units.feetToMeters(1.4);
    public static final double maxLinearAccel = 8.0;

    public static final double maxAngularVelocity = 20;
    public static final double maxAngularAccel = 10;

    public static double kPDriveReal = 2.0; // 1.4866E-05
    public static double kDDriveReal = 0.2;
    public static double kSDriveReal = 0.17236; // 0.097715, 0.027736, 0.021057, 0.093808
    public static double kVDriveReal = (0.10324 + 0.11273 + 0.10143 + 0.10776) / 4;
    public static double kADriveReal = (0.0056151 + 0.0040328 + 0.0077964 + 0.0060387) / 4;

    public static double kPTurnReal = 1.5; // 1.5?
    public static double kDTurnReal = 0.0;

    public static double kPDriveSim = 2.0;
    public static double kDDriveSim = 0.2;
    public static double kSDriveSim = 0.4;
    public static double kVDriveSim = 1.93;
    public static double kADriveSim = 0.25;

    public static double kPTurnSim = 2.5;
    public static double kDTurnSim = 0.0;

    public static double kPDriveReplay = 0.0;
    public static double kDDriveReplay = 0.0;
    public static double kSDriveReplay = 0.0;
    public static double kVDriveReplay = 0.0;
    public static double kADriveReplay = 0.0;

    public static double kPTurnReplay = 0.0;
    public static double kDTurnReplay = 0.0;

    public static final double kALinear = (0.10222 + 0.19369 + 0.18158 + 0.11887) / 4;
    public static final double kAAngular = (0.23623 + 0.23717 + 0.23954 + 0.22678) / 4;
    public static final double angularMOI = mass * trackWidthY / 2 * kAAngular / kALinear; // 20.8
  }

  public static class ControlConstants {
    public static final double deadband = 0.01;
  }

  public static class SimConstants {
    public static final double loopTime = 0.02;
  }
}
