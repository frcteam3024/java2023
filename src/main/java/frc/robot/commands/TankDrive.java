// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveTrain;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDrive(DriveTrain subsystem) {
    driveTrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.coastMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driverLeftAxis = Robot.robotContainer.getCopilotRawAxis(Constants.COPILOT_LEFT_AXIS);
    double driverRightAxis = Robot.robotContainer.getCopilotRawAxis(Constants.COPILOT_RIGHT_AXIS);
    double leftMotorOutputs = 0.5 * driverLeftAxis;
    double rightMotorOutputs = 0.5 * driverRightAxis;
    
    driveTrain.setLeftMotorOutputs(leftMotorOutputs);
    driveTrain.setRightMotorOutputs(rightMotorOutputs);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double zeroSpeed = 0;
    driveTrain.setLeftMotorOutputs(zeroSpeed);
    driveTrain.setRightMotorOutputs(zeroSpeed);
    driveTrain.brakeMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
