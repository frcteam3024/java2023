
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
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import frc.robot.Constants;
import frc.robot.commands.TankDrive;


public class DriveTrain extends SubsystemBase {
  //private TalonSRX intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR_ID);

  // TODO: change to FLspeedMotor
  private CANSparkMax FLmotor = new CANSparkMax(Constants.FL_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax FRmotor = new CANSparkMax(Constants.FR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax BRmotor = new CANSparkMax(Constants.BR_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax BLmotor = new CANSparkMax(Constants.BL_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

  private TalonSRX FLangleMotor = new TalonSRX(Constants.FL_ANGLE_MOTOR_ID);
  private TalonSRX FRangleMotor = new TalonSRX(Constants.FR_ANGLE_MOTOR_ID);
  private TalonSRX BRangleMotor = new TalonSRX(Constants.BR_ANGLE_MOTOR_ID);
  private TalonSRX BLangleMotor = new TalonSRX(Constants.BL_ANGLE_MOTOR_ID);

  private AnalogEncoder FLencoder = new AnalogEncoder(Constants.FL_ENCODER_ID);
  private AnalogEncoder FRencoder = new AnalogEncoder(Constants.FR_ENCODER_ID);
  private AnalogEncoder BLencoder = new AnalogEncoder(Constants.BL_ENCODER_ID);
  private AnalogEncoder BRencoder = new AnalogEncoder(Constants.BR_ENCODER_ID);
  private PIDController pid = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  // docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
  private double[] targetAngles;

  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double[] currentAngles = getAngles();
    SmartDashboard.putNumber("FL angle", currentAngles[0]);
    SmartDashboard.putNumber("FR angle", currentAngles[1]);
    SmartDashboard.putNumber("BL angle", currentAngles[2]);
    SmartDashboard.putNumber("BR angle", currentAngles[3]);

    double[] angleMotorOutputs = new double[4];
    for(int i=0; i<4; i++) {
      double error = currentAngles[i] - targetAngles[i];
      // later: create bool flipped for each for shortest path algorithm
      // wrap errors to between -180 and 180
      angleMotorOutputs[i] = pid.calculate(error, 0);
    }
    setAngleMotorOutputs(angleMotorOutputs);
  }

  //@Override
  public void initDefaultCommand() {
    setDefaultCommand(new TankDrive());
  }

  private void setAngleMotorOutputs(double[] motorOutputs) {
    FLangleMotor.set(ControlMode.PercentOutput, motorOutputs[0]);
    FRangleMotor.set(ControlMode.PercentOutput, motorOutputs[1]);
    BLangleMotor.set(ControlMode.PercentOutput, motorOutputs[2]);
    BRangleMotor.set(ControlMode.PercentOutput, motorOutputs[3]);
  }

  public void setFLspeed(double speed) {
    FLmotor.set(speed);
  }
  public void setFRspeed(double speed) {
    FRmotor.set(speed);
  }
  public void setBLspeed(double speed) {
    BLmotor.set(speed);
  }
  public void setBRspeed(double speed) {
    BRmotor.set(speed);
  }

  public void setRightMotors(double speed) {
    FRmotor.set(speed);
    BRmotor.set(speed);
  }

  public void setAngles(double speed) {
    //
    //
    //
    //
  }
 
  public double[] getAngles() {
    double[] anglesArray = {
    FLencoder.getAbsolutePosition(),
    FRencoder.getAbsolutePosition(),
    BLencoder.getAbsolutePosition(),
    BRencoder.getAbsolutePosition()
    };
    return anglesArray;
  }

  // VECTOR ADDITION
  public static double resultantX(double mag1, double dir1, double mag2, double dir2) {
    double x1 = mag1 * Math.cos(dir1); 
    double x2 = mag1 * Math.cos(dir2); 
    double resultantX = x2-x1;
    return resultantX;
  }

  public static double resultantY(double mag1, double dir1, double mag2, double dir2) {
    double y1 = mag1 * Math.sin(dir1); 
    double y2 = mag1 * Math.sin(dir2); 
    double resultantY = y2-y1;
    return resultantY;
  }

  public static double resultantMagnitude(double mag1, double dir1, double mag2, double dir2) {
    double resultantX = resultantX(mag1, dir1, mag2, dir2);
    double resultantY = resultantY(mag1, dir1, mag2, dir2);
    double resultantMagnitude = Math.sqrt(Math.pow(resultantX,2)+Math.pow(resultantY,2));
    return resultantMagnitude;
  }

  public static double resultantDirection(double mag1, double dir1, double mag2, double dir2) {
    double resultantX = resultantX(mag1, dir1, mag2, dir2);
    double resultantY = resultantY(mag1, dir1, mag2, dir2);
    double resultantDirection = Math.atan2(resultantX,resultantY);
    return resultantDirection;
  }
}