
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
  public double[] readDriverInputs() {
    double driverXAxis = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_X_AXIS);
    double driverYAxis = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_Y_AXIS);
    double driverTwist = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_ROTATE);
    double dirverRawSlider = Robot.m_robotContainer.GetDriverRawAxis(Constants.DRIVE_SLIDER);
    double driverSensitivity = Math.pow(0.5*(1-dirverRawSlider), 2);
    double[] driverInputs = {
      driverXAxis,
      driverYAxis,
      driverTwist,
      driverSensitivity
    };
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
    double[] targetAngles = new double[4];
    double maxMagnitude = 0;

    for(int i=0; i<4; i++) {
      double rotationDirection = Constants.ROTATION_DIRECTIONS[i];
      double[] rotationVector = {rotationMagnitude, rotationDirection};
      double[] resultantVector = resultantVector(translationVector, rotationVector); 
      double resultantMagnitude = driverSensitivity * resultantVector[0];
      double resultantDirection = resultantVector[1];

      targetSpeeds[i] = resultantMagnitude;
      targetAngles[i] = resultantDirection;

      if(resultantMagnitude > maxMagnitude) {
        maxMagnitude = resultantMagnitude;
      }

    };

    // scale speed magnitudes to within [0,1] if any are outside that range
    if(maxMagnitude > 1){
      for(int i=0; i<4; i++){
        targetSpeeds[i] /= maxMagnitude;
      }
    }
    double[][] targetValues = {targetSpeeds, targetAngles};
    return targetValues;

  }
  
  public static double[] resultantVector(double[] vector1, double[] vector2) {
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

  public void printToDash(String[] labels, double[] currentMotorAngles) {
    for(int i=0; i<4; i++) {
      SmartDashboard.putNumber(labels[i], currentMotorAngles[i]);
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
