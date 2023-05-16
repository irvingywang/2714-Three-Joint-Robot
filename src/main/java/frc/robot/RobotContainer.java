// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Manipulator;

public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final Arm m_arm = new Arm();
	private final Manipulator m_manipulator = new Manipulator();

	private static enum CargoType {
		CONE, CUBE
	}

	private static enum IntakeMode {
		SHELF, PORTAL, FLOOR
	}

	private static enum ScoreLevel {
		HIGH, MIDDLE, LOW
	}

	private static enum RobotDirection {
		FORWARD, REVERSE
	}

	private CargoType cargoType = CargoType.CONE;
	private IntakeMode intakeMode = IntakeMode.FLOOR;
	private ScoreLevel scoreLevel = ScoreLevel.HIGH;
	private RobotDirection robotDirection = RobotDirection.FORWARD;

	CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

	Command forwardShelfCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardShelfCone)),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardShelfCube))),
		() -> cargoType);

	Command reverseShelfCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardShelfCone.getInverse())),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardShelfCube.getInverse()))),
		() -> cargoType);

	Command forwardPortalCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardPortalCone)),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardPortalCube))),
		() -> cargoType);

	Command reversePortalCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardPortalCone.getInverse())),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardPortalCube.getInverse()))),
		() -> cargoType);

	Command forwardFloorCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardFloorCone)),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardFloorCube))),
		() -> cargoType);

	Command reverseFloorCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardFloorCone.getInverse())),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardFloorCube.getInverse()))),
		() -> cargoType);

	Command forwardHighCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardHighCone)),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardHighCube))),
		() -> cargoType);

	Command reverseHighCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardMiddleCone.getInverse())),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardMiddleCube.getInverse()))),
		() -> cargoType);

	Command forwardMiddleCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardLowCone)),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardLowCube))),
		() -> cargoType);

	Command reverseMiddleCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardPortalCone.getInverse())),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardPortalCube.getInverse()))),
		() -> cargoType);

	Command forwardLowCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardFloorCone)),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardFloorCube))),
		() -> cargoType);

	Command reverseLowCommand = new SelectCommand(
		Map.ofEntries(
			Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kForwardFloorCone.getInverse())),
			Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kForwardFloorCube.getInverse()))),
		() -> cargoType);

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

	private Command setCargoType(CargoType cargoType) {
		return new InstantCommand(() -> this.cargoType = cargoType);
	}

	private Command setIntakeMode(IntakeMode intakeMode) {
		return new InstantCommand(() -> this.intakeMode = intakeMode);
	}

	private Command setScoreLevel(ScoreLevel scoreLevel) {
		return new InstantCommand(() -> this.scoreLevel = scoreLevel);
	}

	private Command getIntakeCommand() {
		return new SelectCommand(
				Map.ofEntries(
					Map.entry(IntakeMode.SHELF, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, forwardShelfCommand),
							Map.entry(RobotDirection.REVERSE, reverseShelfCommand)),
						() -> robotDirection)),
					Map.entry(IntakeMode.PORTAL, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, forwardPortalCommand),
							Map.entry(RobotDirection.REVERSE, reversePortalCommand)),
						() -> robotDirection)),
					Map.entry(IntakeMode.FLOOR, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, forwardFloorCommand),
							Map.entry(RobotDirection.REVERSE, reverseFloorCommand)),
						() -> robotDirection))
				), () -> intakeMode)
			.alongWith(m_manipulator.setIntake());
	}

	private Command getScoreCommand() {
		return new SelectCommand(
				Map.ofEntries(
					Map.entry(ScoreLevel.HIGH, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, forwardHighCommand),
							Map.entry(RobotDirection.REVERSE, reverseHighCommand)),
						() -> robotDirection)),
					Map.entry(ScoreLevel.MIDDLE, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, forwardMiddleCommand),
							Map.entry(RobotDirection.REVERSE, reverseMiddleCommand)),
						() -> robotDirection)),
					Map.entry(ScoreLevel.LOW, new SelectCommand(
						Map.ofEntries(
							Map.entry(RobotDirection.FORWARD, forwardLowCommand),
							Map.entry(RobotDirection.REVERSE, reverseLowCommand)),
						() -> robotDirection))
				), () -> scoreLevel)
			.andThen(m_manipulator.setOuttake());
	}

	public Command holdCargo() {
		return m_arm.toPreset(ArmConstants.kHoldPreset).alongWith(m_manipulator.setHold());
	}

	private void configureButtonBindings() {
		DriverStation.silenceJoystickConnectionWarning(true);

		m_driverController.rightBumper()
			.onTrue(setCargoType(CargoType.CONE));
		m_driverController.leftBumper()
			.onTrue(setCargoType(CargoType.CUBE));

		m_driverController.povUp()
			.onTrue(setIntakeMode(IntakeMode.SHELF));
		m_driverController.povRight()
			.onTrue(setIntakeMode(IntakeMode.PORTAL));
		m_driverController.povDown()
			.onTrue(setIntakeMode(IntakeMode.FLOOR));

		m_driverController.y()
			.onTrue(setScoreLevel(ScoreLevel.HIGH));
		m_driverController.b()
			.onTrue(setScoreLevel(ScoreLevel.MIDDLE));
		m_driverController.a()
			.onTrue(setScoreLevel(ScoreLevel.LOW));

		m_driverController.leftTrigger().debounce(0.2)
			.onTrue(getIntakeCommand())
			.onFalse(holdCargo());

		m_driverController.rightTrigger().debounce(0.2)
			.onTrue(getScoreCommand())
			.onFalse(holdCargo());
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public void updateTelemetry() {
		SmartDashboard.putString("Cargo Type", cargoType.toString());
		SmartDashboard.putString("Intake Mode", intakeMode.toString());
		SmartDashboard.putString("Score Level", scoreLevel.toString());
	}
}