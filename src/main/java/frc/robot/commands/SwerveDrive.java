// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SwerveDrive extends CommandBase {
  private static final String[] angleLabels = {"FL angle", "FR angle", "BL angle", "BR angle"};
  private static final String[] speedOutputLabels = {"FL speedOutput", "FR speedOutput", "BL speedOutput", "BR speedOutput"};
  private static final String[] angleOutputLabels = {"FL angleOutput", "FR angleOutput", "BL angleOutput", "BR angleOutput"};

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
    double[][] targetValues = Robot.driveTrain.calculateTargetValues(driverInputs);
    double[] targetSpeeds = targetValues[0];
    double[] targetAngles = targetValues[1];

    double[] currentMotorAngles = Robot.driveTrain.getMotorAngles();
    Robot.driveTrain.printToDash(angleLabels, currentMotorAngles);
    
    double[] speedMotorOutputs = targetSpeeds;
    Robot.driveTrain.printToDash(speedOutputLabels, speedMotorOutputs);
    double[] angleMotorOutputs = Robot.driveTrain.calculateAngleMotorOutputs(currentMotorAngles, targetAngles);
    Robot.driveTrain.printToDash(angleOutputLabels, angleMotorOutputs);

    Robot.driveTrain.setAngleMotorOutputs(angleMotorOutputs);
    Robot.driveTrain.setSpeedMotorOutputs(speedMotorOutputs);
    // wrap 0 to 360 degrees: take difference (error), make between -180 and 180
    // pass to pid.calculate
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
