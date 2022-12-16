package frc.robot.commands;

<<<<<<< HEAD
// import static frc.robot.Constants.DriveConstants.MIN_MOTOR_OUTPUT;
// import static java.lang.Math.abs;
=======
import static frc.robot.Constants.OIConstants.MIN_MOTOR_OUTPUT;
import static java.lang.Math.abs;
>>>>>>> b892b3b4576810f0886ee3ffac5a1ef6dba93727

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TankDriveCmd extends CommandBase {

  private DriveSubsystem driveSubsystem;
  private Supplier<Double> leftAxisFunction;
  private Supplier<Double> rightAxisFunction;

  public TankDriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> leftAxisFunction, Supplier<Double> rightAxisFunction) {
    this.driveSubsystem = driveSubsystem;
    this.leftAxisFunction = leftAxisFunction;
    this.rightAxisFunction = rightAxisFunction;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.brakeMode();
  }

  @Override
  public void execute() {
    double driverLeftAxis = leftAxisFunction.get();
    double driverRightAxis = rightAxisFunction.get();
    double leftMotorOutputs = driverLeftAxis;
    double rightMotorOutputs = -driverRightAxis;

<<<<<<< HEAD
    // if (abs(leftMotorOutputs)  < MIN_MOTOR_OUTPUT) leftMotorOutputs  = 0;
    // if (abs(rightMotorOutputs) < MIN_MOTOR_OUTPUT) rightMotorOutputs = 0;
=======
    //if (abs(leftMotorOutputs)  < MIN_MOTOR_OUTPUT) leftMotorOutputs  = 0;
    //if (abs(rightMotorOutputs) < MIN_MOTOR_OUTPUT) rightMotorOutputs = 0;
>>>>>>> b892b3b4576810f0886ee3ffac5a1ef6dba93727
    
    SmartDashboard.putNumber("leftOut", leftMotorOutputs);
    SmartDashboard.putNumber("rightOut", rightMotorOutputs);
    driveSubsystem.setLeftMotorOutputs(leftMotorOutputs);
    driveSubsystem.setRightMotorOutputs(rightMotorOutputs);
  }

  @Override
  public void end(boolean interrupted) {
    double zeroSpeed = 0;
    driveSubsystem.setLeftMotorOutputs(zeroSpeed);
    driveSubsystem.setRightMotorOutputs(zeroSpeed);
    driveSubsystem.coastMode();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
