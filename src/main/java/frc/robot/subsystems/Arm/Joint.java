// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.controller.AsymmetricProfiledPIDController;
import frc.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.utils.controller.AsymmetricTrapezoidProfile.State;

public class Joint extends SubsystemBase {
  private CANSparkMax JointMotor;
  private AbsoluteEncoder JointEncoder;

  private AsymmetricProfiledPIDController JointController;
  private ArmFeedforward JointFeedForward;

  private double JointGearRatio;
  private double KinematicOffset;

  /** Creates a new Joint. */
  public Joint() {}

  public void configJoint(CANSparkMax JointMotor, AbsoluteEncoder JointEncoder,
      AsymmetricProfiledPIDController JointController, ArmFeedforward JointFeedForward, double JointGearRatio,
      double KinematicOffset) {
    this.JointMotor = JointMotor;
    this.JointEncoder = JointEncoder;

    this.JointController = JointController;
    this.JointFeedForward = JointFeedForward;

    this.JointGearRatio = JointGearRatio;
    this.KinematicOffset = KinematicOffset;
  }

  public double getKinematicAngle() {
    return (JointEncoder.getPosition() - KinematicOffset) / JointGearRatio;
  }

  public void setTargetKinematicAngleRadians(double targetAngleRadians) {
    JointController.setGoal(new State(targetAngleRadians, 0));
  }

  public void setConstraints(Constraints constraints) {
    JointController.setConstraints(constraints);
  }

  public boolean atGoal() {
    return JointController.atGoal();
  }

  public void setCalculatedVoltage() {
    JointMotor.setVoltage(JointController.calculate(getKinematicAngle())
        + JointFeedForward.calculate(JointController.getSetpoint().position, 0));
  }
}