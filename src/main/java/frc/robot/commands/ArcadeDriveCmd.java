package frc.robot.commands;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.pow;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCmd extends CommandBase {

  private DriveSubsystem driveTrain;
  private Supplier<Double> speedFunction;
  private Supplier<Double> turnFunction;

  public ArcadeDriveCmd(DriveSubsystem driveTrain, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    this.driveTrain = driveTrain;
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    driveTrain.brakeMode();
  }

  @Override
  public void execute() {
    double driverSpeed = speedFunction.get();
    double driverTurn = turnFunction.get();

    double leftMotorOutputs  = driverSpeed + pow(driverTurn,3);
    double rightMotorOutputs = driverSpeed - pow(driverTurn,3);

    double outputMax = max(abs(leftMotorOutputs), abs(rightMotorOutputs));

    if (outputMax > 1) {
      leftMotorOutputs  /= outputMax;
      rightMotorOutputs /= outputMax;
    }

    if (abs(leftMotorOutputs)  < 0.05) leftMotorOutputs  = 0;
    if (abs(rightMotorOutputs) < 0.05) rightMotorOutputs = 0;
    
    SmartDashboard.putNumber("leftOut", leftMotorOutputs);
    SmartDashboard.putNumber("rightOut", rightMotorOutputs);
    driveTrain.setLeftMotorOutputs(leftMotorOutputs);
    driveTrain.setRightMotorOutputs(rightMotorOutputs);
  }

  @Override
  public void end(boolean interrupted) {
    final double zeroSpeed = 0;
    driveTrain.setLeftMotorOutputs(zeroSpeed);
    driveTrain.setRightMotorOutputs(zeroSpeed);
    driveTrain.coastMode();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
