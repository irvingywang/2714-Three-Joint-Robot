// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.utils.ArmPreset;
import frc.utils.controller.AsymmetricTrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ShoulderConstants {
    public static final int kRightShoulderMotorCanId = 10;
    public static final int kLeftShoulderMotorCanId = 9;

    public static final double kShoulderGearRatio = 1.0;
    public static final double kShoulderLength = Units.inchesToMeters(0.0);
    public static final double kShoulderPositionConversionFactor = (2*Math.PI) * kShoulderGearRatio; //Radians * Gear ratio
    public static final double kShoulderEncoderZeroOffset = 0.0;
    public static final double kShoulderKinematicOffset = 0.0;
    public static final boolean kShoulderMotorInverted = true;
    public static final boolean kShoulderEncoderInverted = true;
    public static final double kShoulderP = 0.0;
    public static final int kShoulderMotorCurrentLimit = 40; //amps

    public static final Constraints kFarConstraints = new Constraints(16, 28, 24);
    public static final Constraints kCloseConstraints = new Constraints(30, 40, 30);
  }

  public static final class ElbowConstants {
    public static final int kElbowMotorCanId = 11;

    public static final double kElbowGearRatio = 1.0;
    public static final double kElbowLength = Units.inchesToMeters(0.0);
    public static final double kElbowPositionConversionFactor = (2*Math.PI) * kElbowGearRatio;
    public static final double kElbowEncoderZeroOffset = 0.0;
    public static final double kElbowKinematicOffset = 0.0;
    public static final boolean kElbowMotorInverted = false;
    public static final boolean kElbowEncoderInverted = true;
    public static final double kElbowP = 0.0;
    public static final int kElbowMotorCurrentLimit = 40; //amps

    public static final Constraints kFarConstraints = new Constraints(16, 28, 24);
    public static final Constraints kCloseConstraints = new Constraints(30, 40, 30);
  }

  public static final class WristConstants {
    public static final int kWristMotorCanId = 13;

    public static final double kWristGearRatio = 1.0;
    public static final double kWristLength = Units.inchesToMeters(0);
    public static final double kWristPositionConversionFactor = (2*Math.PI) * kWristGearRatio;
    public static final double kWristEncoderZeroOffset = 0.0;
    public static final double kWristKinematicOffset = 0.0;
    public static final boolean kWristMotorInverted = false;
    public static final boolean kWristEncoderInverted = false;
    public static final double kWristP = 0.0;
    public static final int kWristMotorCurrentLimit = 30; //amps

    public static final Constraints kFarConstraints = new Constraints(16, 28, 24);
    public static final Constraints kCloseConstraints = new Constraints(30, 40, 30);
  }

  public static final class ManipulatorConstants {
    public static final int kManipulatorMotorCanId = 14;
    public static final int kManipulatorMotorCurrentLimit = 12; //amps 

    public static final double kIntakeMotorSpeed = 1;
    public static final double kHoldMotorSpeed = 0.5;
    public static final double kOuttakeMotorSpeed = -0.5;
  }

  public static final class ArmConstants {
    //Intake
    public static final ArmPreset kForwardShelfCone = new ArmPreset(90, 90, 90);
    public static final ArmPreset kForwardShelfCube = new ArmPreset(90, 90, 45);

    public static final ArmPreset kForwardPortalCone = new ArmPreset(90, 90, 0);
    public static final ArmPreset kForwardPortalCube = new ArmPreset(90, 45, 0);

    public static final ArmPreset kForwardFloorCone = new ArmPreset(90, 0, 90);
    public static final ArmPreset kForwardFloorCube = new ArmPreset(90, 0, 45);

    //Score
    public static final ArmPreset kForwardHighCone = new ArmPreset(30, 30, 30);
    public static final ArmPreset kForwardHighCube = new ArmPreset(30, 30, 45);

    public static final ArmPreset kForwardMiddleCone = new ArmPreset(30, 90, 0);
    public static final ArmPreset kForwardMiddleCube = new ArmPreset(30, 45, 0);

    public static final ArmPreset kForwardLowCone = new ArmPreset(30, 0, 90);
    public static final ArmPreset kForwardLowCube = new ArmPreset(30, 0, 45);

    public static final ArmPreset kHoldPreset = new ArmPreset(0, 0, 0);
  }

}
