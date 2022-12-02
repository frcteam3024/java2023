package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static java.lang.Math.*;

public class SwerveJoystickCommand extends CommandBase {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> magnitudeFunction;
  private final Supplier<Double> directionFunction;
  private final Supplier<Double> turnSpeedFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter turnLimiter;

  public SwerveJoystickCommand(SwerveSubsystem swerveSubsystem,
      Supplier<Double> magnitudeFunction, Supplier<Double> directionFunction,
      Supplier<Double> turnSpeedFunction, Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.magnitudeFunction = magnitudeFunction;
    this.directionFunction = directionFunction;
    this.turnSpeedFunction = turnSpeedFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(TELE_DRIVE_MAX_ACCEL_UNITS_PER_SEC);
    this.yLimiter = new SlewRateLimiter(TELE_DRIVE_MAX_ACCEL_UNITS_PER_SEC);
    this.turnLimiter = new SlewRateLimiter(TELE_DRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.brakeMode();
  }
  
  @Override
  public void execute() {
    // get real-time joystick inputs
    double magnitude = magnitudeFunction.get();
    double direction = -directionFunction.get();
    double relativeAngle = IEEEremainder(direction, PI/2);
    double fixedMagnitude = cos(relativeAngle) * magnitude;
    double xSpeed = fixedMagnitude * cos(direction);
    double ySpeed = fixedMagnitude * sin(direction);
    double turnSpeed = turnSpeedFunction.get();

    SmartDashboard.putNumber("turnSpeed1", turnSpeed);
    // apply deadband
    xSpeed = Math.abs(xSpeed) > AXIS_DEADBAND ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > AXIS_DEADBAND ? ySpeed : 0.0;
    turnSpeed = Math.abs(turnSpeed) > TURN_DEADBAND ? turnSpeed : 0.0;
    SmartDashboard.putNumber("turnSpeed2", turnSpeed);

    // make driving smoother with rate limiting
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    turnSpeed = turnLimiter.calculate(turnSpeed);

    // convert to meters per second
    xSpeed *= TELE_DRIVE_MAX_SPEED_METERS_PER_SEC;
    ySpeed *= TELE_DRIVE_MAX_SPEED_METERS_PER_SEC;
    turnSpeed *= TELE_DRIVE_MAX_ANGULAR_SPEED_RAD_PER_SEC;

    // construct desired chassis speeds
    ChassisSpeeds chassisSpeeds = 
      fieldOrientedFunction.get().equals(Boolean.TRUE)
      // relative to field
      ? ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed, ySpeed, turnSpeed, swerveSubsystem.getGyroRotation2d())
      // relative to robot
      : new ChassisSpeeds(ySpeed, xSpeed, turnSpeed);

    // convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    SmartDashboard.putString("chassisSpeeds", chassisSpeeds.toString());
    // output module states to each wheel
    swerveSubsystem.setModuleStates(moduleStates);
    for (int i=0; i<4; i++) {
      SmartDashboard.putNumber("swerve ["+i+"] speed", moduleStates[i].speedMetersPerSecond);
      SmartDashboard.putNumber("swerve ["+i+"] angle", moduleStates[i].angle.getDegrees());
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    swerveSubsystem.coastMode();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
