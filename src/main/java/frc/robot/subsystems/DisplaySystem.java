// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DisplaySystem extends SubsystemBase {
  /** Creates a new DisplaySystem. */
  public DisplaySystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void printValueToDash(String label, double value) {
    if (Constants.DUBUG_MODE)
      SmartDashboard.putNumber(label, value);
  }

  public void printArrayToDash(String label, double[] valueArray) {
    //if (Constants.DUBUG_MODE) {
    //  SmartDashboard.putNumberArray(label, valueArray);
    //}
    for (int i=0; i<4; i++)
      printValueToDash(Robot.driveTrain.driveModules[i].location+label, valueArray[i]);
  }
  
  public void printVectorToDash(String label, double[] vector) {
    if (Constants.DUBUG_MODE)
      SmartDashboard.putNumberArray(label, vector);
  }

  public void printVectorsToDash(String label, double[][] vectorArray) {
    if (Constants.DUBUG_MODE)
      for (int i=0; i<vectorArray.length; i++)
        SmartDashboard.putNumberArray(Robot.driveTrain.driveModules[i].location+" "+label, vectorArray[i]);
  }

  /**
  public void printAnglesToDash() {
    double[] currentMotorAngles = new double[4];
    double[] angleMotorOutputs = new double[4];
    for (int i=0; i<4; i++) {
      currentMotorAngles[i] = Robot.driveTrain.driveModules[i].currentAngle;
      angleMotorOutputs[i] = Robot.driveTrain.driveModules[i].angleMotorOutput;
      String currentModule = Robot.driveTrain.driveModules[i].location;
      printValueToDash(currentModule+" current angle", currentMotorAngles[i]);
      printValueToDash(currentModule+" angle output", angleMotorOutputs[i]);  
    }
  }

  public void printSpeedsToDash() {
    double[] speedMotorOutputs = new double[4];
    for (int i=0; i<4; i++)
      speedMotorOutputs[i] = Robot.driveTrain.driveModules[i].speedMotorOutput;
    printArrayToDash("speed output", speedMotorOutputs);
  }
  */

}
