// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

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

    double[] driverInputs = Robot.driveTrain.readDriverInputs();
    double[][] targetVectors = Robot.driveTrain.calculateTargetVectors(driverInputs);

    double[] targetSpeeds = new double[4];
    double[] targetAngles = new double[4];
    for (int i=0; i<4; i++) {
      targetSpeeds[i] = targetVectors[i][0];
      targetAngles[i] = targetVectors[i][1];
    }
    Robot.driveTrain.setMotorSpeeds(targetSpeeds);
    Robot.driveTrain.setMotorAngles(targetAngles);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double[] zeroSpeed = {0,0,0,0};
    Robot.driveTrain.setMotorSpeeds(zeroSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
