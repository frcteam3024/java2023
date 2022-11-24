package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import static frc.robot.Constants.OIConstants.*;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX intakeMotor = new TalonSRX(INTAKE_MOTOR_ID);

  public IntakeSubsystem() { /* TODO document why this constructor is empty */ }

  @Override
  public void periodic() { /* TODO document why this method is empty */ }

  public void setIntakeMotor(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

}
