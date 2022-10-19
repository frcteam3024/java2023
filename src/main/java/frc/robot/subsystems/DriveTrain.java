// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  
  private CANSparkMax motorLeft1 = new CANSparkMax(Constants.MOTOR_LEFT_1_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANSparkMax motorLeft2 = new CANSparkMax(Constants.MOTOR_LEFT_2_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private TalonSRX motorRight1 = new TalonSRX(Constants.MOTOR_RIGHT_1_ID);
  private TalonSRX motorRight2 = new TalonSRX(Constants.MOTOR_RIGHT_2_ID);

  /** Creates a new DriveTrain. */
  public DriveTrain() { /* TODO document why this constructor is empty */ }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setLeftMotorOutputs(double speed) {
    motorLeft1.set(speed);
    motorLeft2.set(speed);
  }

  public void setRightMotorOutputs(double speed) {
    motorRight1.set(ControlMode.PercentOutput, speed);
    motorRight2.set(ControlMode.PercentOutput, speed);
  }

  public void coastMode() {
    motorLeft1.setIdleMode(IdleMode.kCoast);
    motorLeft2.setIdleMode(IdleMode.kCoast);
    motorRight1.setNeutralMode(NeutralMode.Coast);
    motorRight2.setNeutralMode(NeutralMode.Coast);
  }

  public void brakeMode() {
    motorLeft1.setIdleMode(IdleMode.kBrake);
    motorLeft2.setIdleMode(IdleMode.kBrake);
    motorRight1.setNeutralMode(NeutralMode.Brake);
    motorRight2.setNeutralMode(NeutralMode.Brake);
  }
}
