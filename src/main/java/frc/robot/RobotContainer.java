// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SpinIntakeCommand;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.OIConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final Joystick driverController  = new Joystick(DRIVE_JOYSTICK_PORT);
  private final Joystick copilotController = new Joystick(COPILOT_JOYSTICK_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
      swerveSubsystem,
      driverController::getMagnitude,
      driverController::getDirectionRadians,
      driverController::getTwist,
      () -> !driverController.getRawButton(FIELD_ORIENTED_TOGGLE_BUTTON)));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, RESET_GYRO_BUTTON)
      .whenPressed(swerveSubsystem::zeroHeading);
    new JoystickButton(copilotController, SPIN_INTAKE_BUTTON)
      .whileHeld(new SpinIntakeCommand(intakeSubsystem));
    
  }

  public Command getAutonomousCommand() {
    return null;
  }
  
}
