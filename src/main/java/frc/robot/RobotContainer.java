// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Arm.CargoType;
import frc.robot.subsystems.Arm.Arm.IntakeMode;
import frc.robot.subsystems.Arm.Arm.RobotDirection;
import frc.robot.subsystems.Arm.Arm.ScoreLevel;


public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final Arm m_arm = new Arm();

	CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

	public RobotContainer() {
		configureButtonBindings();

		m_robotDrive.setDefaultCommand(
			new RunCommand(
				() -> m_robotDrive.drive(
					-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
					true, false),
				m_robotDrive));
	}

	private void configureButtonBindings() {
		DriverStation.silenceJoystickConnectionWarning(true);

		m_driverController.rightBumper()
			.onTrue(m_arm.setCargoType(CargoType.CONE));
		m_driverController.leftBumper()
			.onTrue(m_arm.setCargoType(CargoType.CUBE));

		m_driverController.povUp()
			.onTrue(m_arm.setIntakeMode(IntakeMode.SHELF));
		m_driverController.povRight()
			.onTrue(m_arm.setIntakeMode(IntakeMode.PORTAL));
		m_driverController.povDown()
			.onTrue(m_arm.setIntakeMode(IntakeMode.FLOOR));

		m_driverController.y()
			.onTrue(m_arm.setScoreLevel(ScoreLevel.HIGH));
		m_driverController.b()
			.onTrue(m_arm.setScoreLevel(ScoreLevel.MIDDLE));
		m_driverController.a()
			.onTrue(m_arm.setScoreLevel(ScoreLevel.LOW));

		m_driverController.start()
			.onTrue(m_arm.setRobotDirection(RobotDirection.FORWARD));
		m_driverController.back()
			.onTrue(m_arm.setRobotDirection(RobotDirection.REVERSE));

		m_driverController.leftTrigger().debounce(0.2)
			.onTrue(m_arm.getIntakeCommand())
			.onFalse(m_arm.getHoldCommand());
		m_driverController.rightTrigger().debounce(0.2)
			.onTrue(m_arm.getScoreCommand())
			.onFalse(m_arm.getHoldCommand());
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public void updateTelemetry() {
		m_arm.updateTelemetry();
	}
}