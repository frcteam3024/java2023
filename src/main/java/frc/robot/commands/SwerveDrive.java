// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

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
  public void initialize() {
    for (int i=0; i<4; i++) {
      Robot.driveTrain.driveModules[i].speedMotor.setIdleMode(IdleMode.kBrake);
      Robot.driveTrain.driveModules[i].angleMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double[] driverInputs = Robot.driveTrain.readDriverInputs();
    double[][] targetVectors = Robot.driveTrain.calculateTargetVectors(driverInputs);

    double[] targetSpeeds = targetVectors[0];
    double[] targetAngles = targetVectors[1];
    
    //Robot.driveTrain.setMotorSpeeds(targetSpeeds);
    Robot.driveTrain.setMotorAngles(targetAngles);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double[] zeroSpeed = {0,0,0,0};
    Robot.driveTrain.setMotorSpeeds(zeroSpeed);
    for (int i=0; i<4; i++) {
      Robot.driveTrain.driveModules[i].speedMotor.setIdleMode(IdleMode.kCoast);
      Robot.driveTrain.driveModules[i].angleMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
