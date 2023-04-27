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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.WristConstants;
import frc.utils.controller.AsymmetricProfiledPIDController;

public class Wrist extends Joint {
    private CANSparkMax WristMotor = new CANSparkMax(WristConstants.kWristMotorCanId,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private AbsoluteEncoder WristEncoder = WristMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private AsymmetricProfiledPIDController WristController = new AsymmetricProfiledPIDController(0, 0, 0,
            WristConstants.kFarConstraints);

    private ArmFeedforward WristFeedForward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

    /** Creates a new Wrist. */
    public Wrist() {
        WristMotor.setInverted(WristConstants.kWristMotorInverted);
        WristMotor.setIdleMode(IdleMode.kBrake);
        WristMotor.setSmartCurrentLimit(WristConstants.kWristMotorCurrentLimit);

        WristEncoder.setPositionConversionFactor(WristConstants.kWristPositionConversionFactor);
        WristEncoder.setInverted(WristConstants.kWristEncoderInverted);
        WristEncoder.setZeroOffset(WristConstants.kWristEncoderZeroOffset);

        WristMotor.burnFlash();

        WristController.disableContinuousInput();

        configJoint(WristMotor, WristEncoder, WristController, WristFeedForward, WristConstants.kWristGearRatio,
                WristConstants.kWristKinematicOffset);
    }

    @Override
    public void periodic() {
        setCalculatedVoltage();
        SmartDashboard.putNumber("Wrist Kinematic Angle", getKinematicAngle());
    }
}