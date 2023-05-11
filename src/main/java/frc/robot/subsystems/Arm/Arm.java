// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.utils.ArmPreset;

/** Add your docs here. */
public class Arm {
    private final Shoulder m_shoulder = new Shoulder();
    private final Elbow m_elbow = new Elbow();
    private final Wrist m_wrist = new Wrist();

    public Arm() {}

    public CommandBase toPreset(ArmPreset armPreset) {
        CommandBase armCommand = new SequentialCommandGroup(
            new InstantCommand(() -> {
                m_shoulder.setTargetRadians(armPreset.ShoulderAngleRadians);
                m_elbow.setTargetRadians(armPreset.ElbowAngleRadians);
                m_wrist.setTargetRadians(armPreset.WristAngleRadians);
            }),
            new WaitUntilCommand(() -> m_shoulder.atGoal() && m_elbow.atGoal() && m_wrist.atGoal())
        );
        armCommand.addRequirements(m_shoulder, m_elbow, m_wrist);

        return armCommand;
    }
}
