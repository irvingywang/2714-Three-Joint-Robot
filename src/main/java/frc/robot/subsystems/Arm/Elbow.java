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
import frc.robot.Constants.ElbowConstants;
import frc.utils.controller.AsymmetricProfiledPIDController;

public class Elbow extends Joint {
    private CANSparkMax ElbowMotor = new CANSparkMax(ElbowConstants.kElbowMotorCanId,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private AbsoluteEncoder ElbowEncoder = ElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private AsymmetricProfiledPIDController ElbowController = new AsymmetricProfiledPIDController(0, 0, 0,
            ElbowConstants.kFarConstraints);

    private ArmFeedforward ElbowFeedForward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);

    /** Creates a new Elbow. */
    public Elbow() {
        ElbowMotor.setInverted(ElbowConstants.kElbowMotorInverted);
        ElbowMotor.setIdleMode(IdleMode.kBrake);
        ElbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);

        ElbowEncoder.setPositionConversionFactor(ElbowConstants.kElbowPositionConversionFactor);
        ElbowEncoder.setInverted(ElbowConstants.kElbowEncoderInverted);
        ElbowEncoder.setZeroOffset(ElbowConstants.kElbowEncoderZeroOffset);

        ElbowMotor.burnFlash();

        ElbowController.disableContinuousInput();

        configJoint(ElbowMotor, ElbowEncoder, ElbowController, ElbowFeedForward, ElbowConstants.kElbowGearRatio,
                ElbowConstants.kElbowKinematicOffset);
    }

    @Override
    public void periodic() {
        setCalculatedVoltage();
        SmartDashboard.putNumber("Elbow Kinematic Angle", getKinematicAngle());
    }
}