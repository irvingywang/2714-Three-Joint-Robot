// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.utils.ArmPreset;

/** Add your docs here. */
public class Arm {
    private final Shoulder m_shoulder = new Shoulder();
    private final Elbow m_elbow = new Elbow();
    private final Wrist m_wrist = new Wrist();

    public Arm() {}

    public Command setArmPreset(ArmPreset armPreset) {
        return new InstantCommand(() -> {
            m_shoulder.setTargetKinematicAngleRadians(armPreset.ShoulderAngleRadians);
            m_elbow.setTargetKinematicAngleRadians(armPreset.ElbowAngleRadians);
            m_wrist.setTargetKinematicAngleRadians(armPreset.WristAngleRadians);
        });
    }
}
