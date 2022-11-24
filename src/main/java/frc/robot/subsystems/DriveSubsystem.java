package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  private final TalonSRX motorLeft1 = new TalonSRX(MOTOR_LEFT_1_ID);
  private final TalonSRX motorLeft2 = new TalonSRX(MOTOR_LEFT_2_ID);
  private final TalonSRX motorRight1 = new TalonSRX(MOTOR_RIGHT_1_ID);
  private final TalonSRX motorRight2 = new TalonSRX(MOTOR_RIGHT_2_ID);

  public DriveSubsystem() { /* TODO document why this constructor is empty */ }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setLeftMotorOutputs(double speed) {
    motorLeft1.set(ControlMode.PercentOutput, speed);
    motorLeft2.set(ControlMode.PercentOutput, speed);
  }

  public void setRightMotorOutputs(double speed) {
    motorRight1.set(ControlMode.PercentOutput, speed);
    motorRight2.set(ControlMode.PercentOutput, speed);
  }

  public void brakeMode() {
    motorLeft1.setNeutralMode(NeutralMode.Brake);
    motorLeft2.setNeutralMode(NeutralMode.Brake);
    motorRight1.setNeutralMode(NeutralMode.Brake);
    motorRight2.setNeutralMode(NeutralMode.Brake);
  }

  public void coastMode() {
    motorLeft1.setNeutralMode(NeutralMode.Coast);
    motorLeft2.setNeutralMode(NeutralMode.Coast);
    motorRight1.setNeutralMode(NeutralMode.Coast);
    motorRight2.setNeutralMode(NeutralMode.Coast);
  }
}
