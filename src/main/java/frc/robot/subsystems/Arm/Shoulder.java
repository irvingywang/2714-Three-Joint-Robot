// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShoulderConstants;
import frc.utils.controller.AsymmetricProfiledPIDController;

public class Shoulder extends Joint {
        private CANSparkMax LeftShoulderMotor = new CANSparkMax(ShoulderConstants.kLeftShoulderMotorCanId,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private CANSparkMax RightShoulderMotor = new CANSparkMax(ShoulderConstants.kRightShoulderMotorCanId,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        private AbsoluteEncoder ShoulderEncoder = LeftShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);

        private AsymmetricProfiledPIDController ShoulderController = new AsymmetricProfiledPIDController(0, 0, 0,
                        ShoulderConstants.kFarConstraints);

        private ArmFeedforward ShoulderFeedForward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

        /** Creates a new Shoulder. */
        public Shoulder() {
                LeftShoulderMotor.follow(RightShoulderMotor, true);
                RightShoulderMotor.setInverted(ShoulderConstants.kShoulderEncoderInverted); // must be inverted
                RightShoulderMotor.setIdleMode(IdleMode.kBrake);
                LeftShoulderMotor.setIdleMode(IdleMode.kBrake);
                RightShoulderMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderMotorCurrentLimit);
                LeftShoulderMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderMotorCurrentLimit);

                ShoulderEncoder = RightShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
                ShoulderEncoder.setPositionConversionFactor(ShoulderConstants.kShoulderPositionConversionFactor);
                ShoulderEncoder.setInverted(ShoulderConstants.kShoulderEncoderInverted); // must be inverted
                ShoulderEncoder.setZeroOffset(ShoulderConstants.kShoulderEncoderZeroOffset);

                ShoulderController.disableContinuousInput();

                configJoint(LeftShoulderMotor, ShoulderEncoder, ShoulderController, ShoulderFeedForward,
                                ShoulderConstants.kShoulderGearRatio,
                                ShoulderConstants.kShoulderKinematicOffset);
        }

        @Override
        public void periodic() {
                setCalculatedVoltage();
                SmartDashboard.putNumber("Shoulder Angle", getAngleRadians());
                SmartDashboard.putNumber("Shoulder Target", Units.radiansToDegrees(getTargetRadians()));
        }
}