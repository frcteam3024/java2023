
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.SwerveDrive;
//import frc.robot.commands.TankDrive;


public class DriveTrain extends SubsystemBase {
  //private TalonSRX intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR_ID);
  private static final String[] moduleLabels = {"FL", "FR", "BL", "BR"};
  //private static final String[] angleLabels = {"FL angle", "FR angle", "BL angle", "BR angle"};
  //private static final String[] speedOutputLabels = {"FL speedOutput", "FR speedOutput", "BL speedOutput", "BR speedOutput"};
  //private static final String[] angleOutputLabels = {"FL angleOutput", "FR angleOutput", "BL angleOutput", "BR angleOutput"};
  //private static final String[] speedVectotrLabels = {"FL speed vecotr", "FR speed vecotr", "BL speed vecotr", "BR speed vecotr"};
  //private static final String[] angleVectorLabels = {"FL angle vector", "FR angle vector", "BL angle vector", "BR angle vector"};
  //private static final String[] resultantVecotrMagLabels = {"FL resultant magnitude", "FR resultant magnitude", "BL resultant magnitude", "BR resultant magnitude"};
  //private static final String[] resultantVecotrDirLabels = {"FL resultant direction", "FR resultant direction", "BL resultant direction", "BR resultant direction"};

  private CANSparkMax[] speedMotors = {
    new CANSparkMax(Constants.FL_SPEED_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
    new CANSparkMax(Constants.FR_SPEED_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
    new CANSparkMax(Constants.BL_SPEED_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless),
    new CANSparkMax(Constants.BR_SPEED_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless)
  };

  private TalonSRX[] angleMotors = {
    new TalonSRX(Constants.FL_ANGLE_MOTOR_ID),
    new TalonSRX(Constants.FR_ANGLE_MOTOR_ID),
    new TalonSRX(Constants.BL_ANGLE_MOTOR_ID),
    new TalonSRX(Constants.BR_ANGLE_MOTOR_ID)
  };

  private AnalogEncoder[] angleEncoders = {
    new AnalogEncoder(Constants.FL_ENCODER_ID),
    new AnalogEncoder(Constants.FR_ENCODER_ID),
    new AnalogEncoder(Constants.BL_ENCODER_ID),
    new AnalogEncoder(Constants.BR_ENCODER_ID)
  };

  private PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  // docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html

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

  //TODO: implement shortest path angles and flipping wheels once current code runs successfully

  public double[] readDriverInputs() {
    double driverXAxis = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_X_AXIS);
    double driverYAxis = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_Y_AXIS);
    double driverTwist = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_ROTATE);
    double dirverRawSlider = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_SLIDER);
    double driverSensitivity = 0.5*(1-dirverRawSlider);
    double[] driverInputs = {
      driverXAxis,
      driverYAxis,
      driverTwist,
      driverSensitivity
    };
    // TODO: remove magic numbers
    for (int i=0; i<4; i++) {
      if (Math.abs(driverInputs[i]) < 0.05) {
        driverInputs[i] = 0;
      }
    }
    if (Math.abs(driverTwist) < 0.2) {
      driverInputs[2] = 0;
    }
    return driverInputs;
  }

  public double[][] calculateTargetValues(double[] driverInputs) {

    double driverXAxis = driverInputs[0];
    double driverYAxis = driverInputs[1];
    double driverTwist = driverInputs[2];
    double driverSensitivity = driverInputs[3];

    double rotationMagnitude = driverTwist;
    double translationDirection = Math.toDegrees(Math.atan2(driverXAxis, driverYAxis));
    double translationMagnitude = Math.sqrt(Math.pow(driverXAxis,2) + Math.pow(driverYAxis,2));
    double[] translationVector = {translationMagnitude, translationDirection};

    double[] targetSpeeds = new double[4];
    double[] offsetAdjustedAngles = new double[4];
    double[] targetAngles = new double[4];
    double[][] resultantVectors = new double[4][2];
    double[][] rotationVectors = new double[4][2];

    for(int i=0; i<4; i++) {
      double rotationDirection = Constants.ROTATION_DIRECTIONS[i];
      rotationVectors[i][0] = rotationMagnitude;
      rotationVectors[i][1] = rotationDirection;
      resultantVectors[i] = resultantVector(translationVector, rotationVectors[i]); 
      double resultantMagnitude = driverSensitivity * resultantVectors[i][0];
      double resultantDirection = resultantVectors[i][1];

      targetSpeeds[i] = resultantMagnitude;
    //  offsetAdjustedAngles[i] = resultantDirection - Constants.SWERVE_OFFSETS[i];
      targetAngles[i] = resultantDirection;
    }
    
    SmartDashboard.putNumber("translation mag ", translationMagnitude);
    SmartDashboard.putNumber("translation dir ", translationDirection);
    for (int i=0; i<4; i++) {
      SmartDashboard.putNumber("rotation magnitude"+moduleLabels[i], rotationVectors[i][0]);
      SmartDashboard.putNumber("rotation direction"+moduleLabels[i], rotationVectors[i][1]);
    }
    
    
    double[][] targetValues = {targetSpeeds, targetAngles};
    return targetValues;

  }
  
  public void setMotorAngles(double[] targetAngles){
    //double[] normalizedAngles = normalizeAngles(targetAngles);
    double[] currentMotorAngles = getMotorAngles();
    double[] angleMotorOutputs = calculateAngleMotorOutputs(currentMotorAngles, targetAngles);
    if(Constants.DUBUG_MODE){
      printArrayToDash("current angle", currentMotorAngles);
      printArrayToDash("angle outputs", angleMotorOutputs);
    }
    setAngleMotorOutputs(angleMotorOutputs);
  }

  public void setMotorSpeeds(double[] targetSpeeds){
    double[] normalizedSpeeds = normalizeSpeeds(targetSpeeds);
    double[] speedMotorOutputs = normalizedSpeeds;
    if(Constants.DUBUG_MODE){
      printArrayToDash("speed outputs", speedMotorOutputs);
    }
    setSpeedMotorOutputs(speedMotorOutputs);
  }

  private static double[] resultantVector(double[] vector1, double[] vector2) {
  
    double mag1 = vector1[0];
    double dir1 = vector1[1];
    double mag2 = vector2[0];
    double dir2 = vector2[1];

    double x1 = mag1 * Math.cos(dir1); 
    double x2 = mag1 * Math.cos(dir2); 
    double y1 = mag2 * Math.sin(dir1); 
    double y2 = mag2 * Math.sin(dir2); 

    double resultantX = x1+x2;
    double resultantY = y1+y2;
    double resultantMagnitude = Math.sqrt(Math.pow(resultantX,2)+Math.pow(resultantY,2));
    double resultantDirection = Math.atan2(resultantX,resultantY);
    double[] vector = {resultantMagnitude, resultantDirection};
    return vector;
  }

  /**  private double[] shortestPathAngles(double[] targetAngles){

    double[] shortestPathAngles = new double[4];
    //boolean[] flippedWheels = new boolean[4];
    double flipThreshold = Constants.ANGLE_FLIP_THRESHOLD;
    for(int i=0; i<4; i++){
      double normalizedAngle;
      double angleMod = targetAngles[i] % 360;
      if(angleMod <= flipThreshold){
        normalizedAngle = angleMod;
        //flippedWheels[i] = false;
      }else if(flipThreshold < angleMod && angleMod < 360 - flipThreshold){
        normalizedAngle = 180 - angleMod;
        //flippedWheels[i] = true;
      }else{
        normalizedAngle = angleMod - 360;
        //flippedWheels[i] = false;
      }
      shortestPathAngles[i] = normalizedAngle;
    }
    return shortestPathAngles;
  }
  */
  
  private static double[] normalizeSpeeds(double[] targetSpeeds){
    double maxSpeed = 0;
    for(int i=0; i<4; i++){
      if(targetSpeeds[i] > maxSpeed) {
        maxSpeed = targetSpeeds[i];
      }

    };
    double[] normalizedSpeeds = new double[4];
    // scale speed magnitudes to within [-1,1] if any are outside that range
    for (int i=0; i<4; i++) {
      if (maxSpeed > 1) {
        normalizedSpeeds[i] = targetSpeeds[i] / maxSpeed;
      } else {
        normalizedSpeeds[i] = targetSpeeds[i];
      }
      //if (normalizedSpeeds[i] < 0.01) {
      //  normalizedSpeeds[i] = 0;
      //}
      //if(flippedWheels[i]){
      //  normalizedSpeeds[i] *= -1;
      //}
    }
    return normalizedSpeeds;
  }

  public void printArrayToDash(String label, double[] values) {
    for(int i=0; i<4; i++) {
      SmartDashboard.putNumber(label+moduleLabels[i], values[i]);
    }
  }
  
  public void printVectorsToDash(String label, double[][] vector) {
    for(int i=0; i<4; i++) {
      SmartDashboard.putNumber(label+moduleLabels[i]+"mag", vector[i][0]);
      SmartDashboard.putNumber(label+moduleLabels[i]+"dir", vector[i][1]);
    }
  }

  
  public double[] getMotorAngles() {
    double[] motorAngles = new double[4];
    for(int i=0; i<4; i++) {
      motorAngles[i] = angleEncoders[i].getAbsolutePosition();
    };
    return motorAngles;
  }

  public double[] calculateAngleMotorOutputs(double[] currentMotorAngles, double[] targetAngles) {
    double[] angleMotorOutputs = new double[4];
    for(int i=0; i<4; i++) {
      double currentError = currentMotorAngles[i] - targetAngles[i];
      // later: create bool flipped for each for shortest path algorithm
      // wrap errors to between -180 and 180

      SmartDashboard.putNumber(moduleLabels[i]+" angle error", currentError);
      angleMotorOutputs[i] = pid.calculate(currentError);
    }
    return angleMotorOutputs;
  }

  public void setAngleMotorOutputs(double[] angleMotorOutputs) {
    for(int i=0; i<4; i++) {
      angleMotors[i].set(ControlMode.PercentOutput, angleMotorOutputs[i]);
    }
  }

  public void setSpeedMotorOutputs(double[] speedMotorOutputs) {
    for(int i=0; i<4; i++) {
      speedMotors[i].set(speedMotorOutputs[i]);
    }
  }
}
