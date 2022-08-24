// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants;

public class InitMotor extends CommandBase {
  /** Creates a new InitMotor. */
  public InitMotor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.spinMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawSlider = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_SLIDER);
    final double motorSpeed = rawSlider * Robot.m_robotContainer.GetCopilotRawAxis(Constants.COPILOT_LEFT_STICK_Y);
    Robot.spinMotor.setIntakeMotor(motorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.spinMotor.setIntakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
