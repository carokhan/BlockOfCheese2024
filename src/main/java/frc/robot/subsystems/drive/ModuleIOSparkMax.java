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

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(RobotMap.Drive.frontLeftDrive, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(RobotMap.Drive.frontLeftTurn, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(RobotMap.Drive.frontLeftEncoder);
        absoluteEncoderOffset =
            new Rotation2d(RobotMap.Drive.frontLeftOffset); // MUST BE CALIBRATED
        isTurnMotorInverted = RobotMap.Drive.frontLeftTurnInvert;
        break;
      case 1:
        driveSparkMax = new CANSparkMax(RobotMap.Drive.frontRightDrive, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(RobotMap.Drive.frontRightTurn, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(RobotMap.Drive.frontRightEncoder);
        absoluteEncoderOffset =
            new Rotation2d(RobotMap.Drive.frontRightOffset); // MUST BE CALIBRATED
        isTurnMotorInverted = RobotMap.Drive.frontRightTurnInvert;
        break;
      case 2:
        driveSparkMax = new CANSparkMax(RobotMap.Drive.backLeftDrive, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(RobotMap.Drive.backLeftTurn, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(RobotMap.Drive.backLeftEncoder);
        absoluteEncoderOffset = new Rotation2d(RobotMap.Drive.backLeftOffset); // MUST BE CALIBRATED
        isTurnMotorInverted = RobotMap.Drive.backLeftTurnInvert;
        break;
      case 3:
        driveSparkMax = new CANSparkMax(RobotMap.Drive.backRightDrive, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(RobotMap.Drive.backRightTurn, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(RobotMap.Drive.backRightEncoder);
        absoluteEncoderOffset =
            new Rotation2d(RobotMap.Drive.backRightOffset); // MUST BE CALIBRATED
        isTurnMotorInverted = RobotMap.Drive.backRightTurnInvert;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(DriveConstants.driveCurrent);
    turnSparkMax.setSmartCurrentLimit(DriveConstants.turnCurrent);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = driveEncoder.getPosition();
                  if (driveSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = turnRelativeEncoder.getPosition();
                  if (turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();

    // turnRelativeEncoder.setPosition(
    //     turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DriveConstants.driveRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
            / DriveConstants.driveRatio;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / DriveConstants.turnRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / DriveConstants.turnRatio;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble(
                (Double value) -> Units.rotationsToRadians(value) / DriveConstants.driveRatio)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / DriveConstants.turnRatio))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
