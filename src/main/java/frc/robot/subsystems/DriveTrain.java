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
  /** [TODO] declare motorLeft2, motorRight1, and motorRight2 in similar fashion
   *  motor ids should go in the Constants file */

  public DriveTrain() {/** Creates a new DriveTrain. */}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void coastMode() {
    motorLeft1.setIdleMode(IdleMode.kCoast);
    // [TODO] set remaining motors to coast mode
  }

  public void brakeMode() {
    // [TODO] set all motors to brake mode
  }
  public void setLeftMotorOutputs(double speed) {
    motorLeft1.set(speed);
    // [TODO] set left motor outputs to "speed" using .set()
    // hint: use .set() like you used .setIdleMode above
  }

  public void setRightMotorOutputs(double speed) {
    // [TODO] set right motor outputs to "speed"
  }

}
