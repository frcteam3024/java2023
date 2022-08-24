// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;
import frc.robot.commands.InitMotor;


public class SpinMotor extends SubsystemBase {
  private TalonSRX intakeMotor = new TalonSRX(Constants.INTAKE_MOTOR_ID);
  /** Creates a new SpinMotor. */
  public SpinMotor() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //@Override
  public void initDefaultCommand() {
    setDefaultCommand(new InitMotor());
  }


  public void setIntakeMotor(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

}
