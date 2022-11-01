// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonSRX intakeMotor = new TalonSRX(IntakeConstants.INTAKE_MOTOR_ID);

  public IntakeSubsystem() {
    // creates a new IntakeSubsystem
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinIntakeMotor() {
      intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.MAX_INTAKE_SPEED);
  }

  public void stopIntakeMotor() {
      intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void brakeMode() {
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void coastMode() {
    intakeMotor.setNeutralMode(NeutralMode.Coast);
  }

}
