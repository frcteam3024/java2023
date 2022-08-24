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
    double translationDirection = Math.toDegrees(Math.atan2(driverXAxis, driverYAxis));
    double translationMagnitude = Math.sqrt(Math.pow(driverXAxis,2) + Math.pow(driverYAxis,2));
    double[] translationVector = {translationMagnitude, translationDirection};

    double rawSlider = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_SLIDER);
    double driverSensitivity = 0.5 * (1-rawSlider);
 
    double[] targetSpeeds = new double[4];
    double[] targetAngles = new double[4];
    for(int i=0; i<4; i++) {
      double rotationDirection = Constants.ROTATION_DIRECTIONS[i];
      double[] rotationVector = {rotationMagnitude, rotationDirection};
      double[] resultantVector = DriveTrain.resultantVector(translationVector, rotationVector); 
      double resultantMagnitude = driverSensitivity * resultantVector[0];
      double resultantDirection = driverSensitivity * resultantVector[1];
      targetSpeeds[i] = resultantMagnitude;
      targetAngles[i] = resultantDirection;
    };
 
    Robot.driveTrain.setSpeedMotorOutputs(targetSpeeds);
    
    // wrap 0 to 360 degrees: take difference (error), make between -180 and 180
    // pass to pid.calculate

    // call setAngles() from this file
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
