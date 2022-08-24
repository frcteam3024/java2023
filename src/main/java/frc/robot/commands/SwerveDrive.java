// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driverXAxis = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_X_AXIS);
    double driverYAxis = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_Y_AXIS);
    double rotationMagnitude = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_ROTATE);
    double translationDirection;
    if (driverXAxis != 0) {
      translationDirection = Math.toDegrees(Math.atan(driverYAxis/driverXAxis));
    } else {
      translationDirection = driverYAxis;
    }
    double translationMagnitude = Math.sqrt(Math.pow(driverXAxis,2) + Math.pow(driverYAxis,2));

    double rawSlider = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_SLIDER);
    double driverSensitivity = 0.5 * (1-rawSlider);

    double FRspeed = driverSensitivity * DriveTrain.resultantMagnitude(translationMagnitude, translationDirection, rotationMagnitude, Constants.FR_RotationDirection);
    double FLspeed = driverSensitivity * DriveTrain.resultantMagnitude(translationMagnitude, translationDirection, rotationMagnitude, Constants.FL_RotationDirection);
    double BRspeed = driverSensitivity * DriveTrain.resultantMagnitude(translationMagnitude, translationDirection, rotationMagnitude, Constants.BR_RotationDirection);
    double BLspeed = driverSensitivity * DriveTrain.resultantMagnitude(translationMagnitude, translationDirection, rotationMagnitude, Constants.BL_RotationDirection);

    double FRangle = DriveTrain.resultantDirection(translationMagnitude, translationDirection, rotationMagnitude, Constants.FR_RotationDirection);
    double FLangle = DriveTrain.resultantDirection(translationMagnitude, translationDirection, rotationMagnitude, Constants.FL_RotationDirection);
    double BRangle = DriveTrain.resultantDirection(translationMagnitude, translationDirection, rotationMagnitude, Constants.BR_RotationDirection);
    double BLangle = DriveTrain.resultantDirection(translationMagnitude, translationDirection, rotationMagnitude, Constants.BL_RotationDirection);

    Robot.driveTrain.setFLspeed(FLspeed);
    Robot.driveTrain.setFRspeed(FRspeed);
    Robot.driveTrain.setBLspeed(BLspeed);
    Robot.driveTrain.setBRspeed(BRspeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setFLspeed(0);
    Robot.driveTrain.setFRspeed(0);
    Robot.driveTrain.setBLspeed(0);
    Robot.driveTrain.setBRspeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
