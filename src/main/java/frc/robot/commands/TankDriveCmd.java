package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private DriveSubsystem driveTrain;
  private Supplier<Double> leftAxisFunction;
  private Supplier<Double> rightAxisFunction;

  public TankDriveCmd(DriveSubsystem driveTrain, Supplier<Double> leftAxisFunction, Supplier<Double> rightAxisFunction) {
    this.driveTrain = driveTrain;
    this.leftAxisFunction = leftAxisFunction;
    this.rightAxisFunction = rightAxisFunction;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.brakeMode();
  }

  @Override
  public void execute() {
    double driverLeftAxis = leftAxisFunction.get();
    double driverRightAxis = rightAxisFunction.get();
    double leftMotorOutputs = driverLeftAxis;
    double rightMotorOutputs = -driverRightAxis;

    leftMotorOutputs = Math.abs(leftMotorOutputs) < 0.05 ? 0 : leftMotorOutputs;
    rightMotorOutputs = Math.abs(rightMotorOutputs) < 0.05 ? 0 : rightMotorOutputs;
    
    SmartDashboard.putNumber("leftOut", leftMotorOutputs);
    SmartDashboard.putNumber("rightOut", rightMotorOutputs);
    driveTrain.setLeftMotorOutputs(leftMotorOutputs);
    driveTrain.setRightMotorOutputs(rightMotorOutputs);
  }

  @Override
  public void end(boolean interrupted) {
    double zeroSpeed = 0;
    driveTrain.setLeftMotorOutputs(zeroSpeed);
    driveTrain.setRightMotorOutputs(zeroSpeed);
    driveTrain.coastMode();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
