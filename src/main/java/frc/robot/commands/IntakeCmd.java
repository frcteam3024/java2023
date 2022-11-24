package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends CommandBase {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeSubsystem intakeSystem;
  private Supplier<Double> leftAxisFunction;

  public IntakeCmd(IntakeSubsystem intakeSystem, Supplier<Double> leftAxisFunction) {
    this.intakeSystem = intakeSystem;
    this.leftAxisFunction = leftAxisFunction;
    addRequirements(intakeSystem);
  }
  
  double motorSpeed;
  
  @Override
  public void initialize() { /* TODO document why this method is empty */ }

  @Override
  public void execute() {
    motorSpeed = leftAxisFunction.get();
    intakeSystem.setIntakeMotor(motorSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSystem.setIntakeMotor(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
