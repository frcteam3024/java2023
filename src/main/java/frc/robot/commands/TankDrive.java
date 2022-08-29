// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  public TankDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driverYAxis = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_Y_AXIS);
    double rawSlider = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_SLIDER);
    double driverSensitivity = 0.5 * (1-rawSlider);
    double targetSpeed = driverYAxis * driverSensitivity;
    
    double[] speedMotorOutputs = new double[4];
    
    Arrays.fill(speedMotorOutputs, targetSpeed);

    Robot.driveTrain.setSpeedMotorOutputs(speedMotorOutputs);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double[] zeroSpeed = {0,0,0,0};
    Robot.driveTrain.setSpeedMotorOutputs(zeroSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
