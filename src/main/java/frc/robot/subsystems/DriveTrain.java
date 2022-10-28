
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveModule;
import frc.robot.commands.SwerveDrive;
public class DriveTrain extends SubsystemBase {
  DriveModule FLdriveModule = new DriveModule("FL", Constants.FL_MODULE_ID);
  DriveModule FRdriveModule = new DriveModule("FR", Constants.FR_MODULE_ID);
  DriveModule BLdriveModule = new DriveModule("BL", Constants.BL_MODULE_ID);
  DriveModule BRdriveModule = new DriveModule("BR", Constants.BR_MODULE_ID);

  DriveModule[] driveModules = {
    FLdriveModule,
    FRdriveModule,
    BLdriveModule,
    BRdriveModule
  };

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //@Override
  public void initDefaultCommand() {
    setDefaultCommand(new SwerveDrive());
  }
  
  public void coastMode() {
    for (int i=0; i<4; i++) {
      Robot.driveTrain.driveModules[i].speedMotor.setIdleMode(IdleMode.kCoast);
      Robot.driveTrain.driveModules[i].angleMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void brakeMode() {
    for (int i=0; i<4; i++) {
      Robot.driveTrain.driveModules[i].speedMotor.setIdleMode(IdleMode.kBrake);
      Robot.driveTrain.driveModules[i].angleMotor.setNeutralMode(NeutralMode.Brake);
    }
  }

  ///////////////////////////////
  // ANGLE-SPECIFIC FUNCTIONS: //
  ///////////////////////////////

  public void setMotorAngles(double[] targetAngles) {
    //double[] normalizedAngles = normalizeAngles(targetAngles);
    double[] currentMotorAngles = getCurrentMotorAngles();
    double[] angleMotorOutputs = calculateAngleMotorOutputs(currentMotorAngles, targetAngles);
    setAngleMotorOutputs(angleMotorOutputs);
    //Robot.displaySystem.printAnglesToDash();
    Robot.displaySystem.printArrayToDash(" taget angle", targetAngles);
    Robot.displaySystem.printArrayToDash(" current angle", currentMotorAngles);
    Robot.displaySystem.printArrayToDash(" angle output", angleMotorOutputs);
  }

  public double[] getCurrentMotorAngles() {
    // outputs in degrees
    double[] currentMotorAngles = new double[4];
    for (int i=0; i<4; i++)
      currentMotorAngles[i] = driveModules[i].getAngleEncoder();
    return currentMotorAngles;
  }

  private double[] calculateAngleMotorOutputs(double[] currentMotorAngles, double[] targetAngles) {
    double[] angleMotorOutputs = new double[4];
    for (int i=0; i<4; i++) {
      double currentDegError =  targetAngles[i] - currentMotorAngles[i];
      double standardizedDegError = standardizeAngle(currentDegError);
      double finalError = standardizedDegError / 360.0;
      angleMotorOutputs[i] = Constants.kP * finalError;

      Robot.displaySystem.printValueToDash(Robot.driveTrain.driveModules[i].location + " error", standardizedDegError);
      Robot.displaySystem.printValueToDash(Robot.driveTrain.driveModules[i].location + " before kP", finalError);
      Robot.displaySystem.printValueToDash(Robot.driveTrain.driveModules[i].location + " after kP", angleMotorOutputs[i]);
    }
    return angleMotorOutputs;
  }

  private void setAngleMotorOutputs(double[] angleMotorOutputs) {
    for (int i=0; i<4; i++) {
      if (Math.abs(angleMotorOutputs[i]) < 0.05)
        angleMotorOutputs[i] = 0.0;
      //driveModules[i].angleMotorOutput = angleMotorOutputs[i];
      driveModules[i].angleMotor.set(ControlMode.PercentOutput, angleMotorOutputs[i]);
    }
  }

   public double standardizeAngle(double angle) {
    return ((angle + 180.0) % 360.0) - 180.0;
   }
  
}