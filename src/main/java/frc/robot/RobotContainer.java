// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Manipulator;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
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

  private CargoType cargoType = CargoType.CONE;
  private IntakeMode intakeMode = IntakeMode.FLOOR;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
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

  private Command getIntakeCommand() {
    Command shelfCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kShelfPresetCone)),
        Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kShelfPresetCube))
    ), () -> cargoType);

    Command portalCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kPortalPresetCone)),
        Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kPortalPresetCube))
    ), () -> cargoType);

    Command floorCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(CargoType.CONE, m_arm.toPreset(ArmConstants.kFloorPresetCone)),
        Map.entry(CargoType.CUBE, m_arm.toPreset(ArmConstants.kFloorPresetCube))
    ), () -> cargoType);

    return new SelectCommand(
      Map.ofEntries(
        Map.entry(IntakeMode.SHELF, shelfCommand),
        Map.entry(IntakeMode.PORTAL, portalCommand),
        Map.entry(IntakeMode.FLOOR, floorCommand)
    ), () -> intakeMode)
    .alongWith(m_manipulator.setIntake());
  }

  public Command holdCargo() {
    return m_arm.toPreset(ArmConstants.kHoldPreset).alongWith(m_manipulator.setHold());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
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
    
    m_driverController.rightTrigger()
      .onTrue(getIntakeCommand())
      .onFalse(holdCargo());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }

  public void updateTelemetry() {
    SmartDashboard.putString("Cargo Type", cargoType.toString());
    SmartDashboard.putString("Intake Mode", intakeMode.toString());
  }
}
