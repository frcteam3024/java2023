
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
import frc.robot.commands.SwerveDrive;
//import frc.robot.commands.TankDrive;


public class DriveTrain extends SubsystemBase {
  //private TalonSRX intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR_ID);

  private String[] angleNames = {"FL", "FR", "BL", "BR"};

  private CANSparkMax FLspeedMotor = new CANSparkMax(Constants.FL_SPEED_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax FRspeedMotor = new CANSparkMax(Constants.FR_SPEED_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax BLspeedMotor = new CANSparkMax(Constants.BL_SPEED_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax BRspeedMotor = new CANSparkMax(Constants.BR_SPEED_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private TalonSRX FLangleMotor = new TalonSRX(Constants.FL_ANGLE_MOTOR_ID);
  private TalonSRX FRangleMotor = new TalonSRX(Constants.FR_ANGLE_MOTOR_ID);
  private TalonSRX BLangleMotor = new TalonSRX(Constants.BL_ANGLE_MOTOR_ID);
  private TalonSRX BRangleMotor = new TalonSRX(Constants.BR_ANGLE_MOTOR_ID);

  private AnalogEncoder FLangleEncoder = new AnalogEncoder(Constants.FL_ENCODER_ID);
  private AnalogEncoder FRangleEncoder = new AnalogEncoder(Constants.FR_ENCODER_ID);
  private AnalogEncoder BLangleEncoder = new AnalogEncoder(Constants.BL_ENCODER_ID);
  private AnalogEncoder BRangleEncoder = new AnalogEncoder(Constants.BR_ENCODER_ID);

  private CANSparkMax[] speedMotors = {
    FLspeedMotor,
    FRspeedMotor,
    BLspeedMotor,
    BRspeedMotor
  };

  private TalonSRX[] angleMotors = {
    FLangleMotor,
    FRangleMotor,
    BLangleMotor,
    BRangleMotor
  };

  private AnalogEncoder[] angleEncoders = {
    FLangleEncoder,
    FRangleEncoder,
    BLangleEncoder,
    BRangleEncoder
  };

  private PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  // docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
  private double[] targetAngles;

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double[] currentAngles = getAngles();
    printCurrentAngles(currentAngles);

    double[] angleMotorOutputs = new double[4];
    for(int i=0; i<4; i++) {
      double currentError = currentAngles[i] - targetAngles[i];
      // later: create bool flipped for each for shortest path algorithm
      // wrap errors to between -180 and 180

      angleMotorOutputs[i] = pid.calculate(currentError);
    }
    setAngleMotorOutputs(angleMotorOutputs);
  }

  //@Override
  public void initDefaultCommand() {
    setDefaultCommand(new SwerveDrive());
  }

  private void printCurrentAngles(double[] currentAngles) {
    for(int i=0; i<4; i++) {
      SmartDashboard.putNumber(angleNames[i]+" angle", currentAngles[i]);
    }
  }
  
  private void setAngleMotorOutputs(double[] motorOutputs) {
    for(int i=0; i<4; i++) {
      angleMotors[i].set(ControlMode.PercentOutput, motorOutputs[i]);
    }
  }

  public void setSpeedMotorOutputs(double[] targetSpeeds) {
    for(int i=0; i<4; i++) {
      speedMotors[i].set(targetSpeeds[i]);
    }
  }

  public void setRightMotors(double speed) {
    FRspeedMotor.set(speed);
    BRspeedMotor.set(speed);
  }

  public void setAngles(double speed) {
    //
    //
    //
    //
  }
 
  public double[] getAngles() {
    double[] anglesArray = new double[4];
    for(int i=0; i<4; i++) {
      anglesArray[i] = angleEncoders[i].getAbsolutePosition();
    };
    return anglesArray;
  }

  // VECTOR ADDITION
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

}