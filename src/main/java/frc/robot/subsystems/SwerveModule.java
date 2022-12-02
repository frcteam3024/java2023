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
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static java.lang.Math.PI;
import static frc.robot.Constants.ModuleConstants.*;

/** Add your docs here. */
public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final TalonSRX turnMotor;
    
    private final RelativeEncoder driveEncoder;
    // private final /** ? */ turnEncoder;

    private final PIDController turnPIDController;

    private final AnalogEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReversed, boolean turnMotorReversed,
        int absoluteEncoderID, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed) {

      this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      absoluteEncoder = new AnalogEncoder(absoluteEncoderID);

      driveMotor = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
      turnMotor = new TalonSRX(turnMotorID);
      
      driveMotor.setInverted(driveMotorReversed);
      turnMotor.setInverted(turnMotorReversed);

      driveEncoder = driveMotor.getEncoder();
      // turnEncoder = turnMotor.getSelectedSensorPosition(); //?

      driveEncoder.setPositionConversionFactor(DRIVE_ENCODER_ROTATION_TO_METER);
      driveEncoder.setVelocityConversionFactor(DRIVE_ENCODER_RPM_TO_METERS_PER_SEC);

      turnPIDController = new PIDController(KP_TURN, KI_TURN, KD_TURN);
      turnPIDController.enableContinuousInput(-PI, PI);
    
      resetEncoders();
    }

    public double getDrivePosition() {
      return driveEncoder.getPosition();
    }

    public double getTurnPosition() {
      return turnMotor.getSelectedSensorPosition();
    }

    public double getDriveVelocity() {
      return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
      return turnMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteEncoderRad() {
      double angle = absoluteEncoder.getAbsolutePosition();
      angle *= PI;
      angle -= absoluteEncoderOffsetRad;
      angle += (angle < -PI ? 2*PI : 0);
      angle *= (absoluteEncoderReversed ? -1.0 : 1.0);
      return angle;
    }

    public void resetEncoders() {
      driveEncoder.setPosition(0);
      // turnEncoder.setPosition(getAbsoluteEncoderRad());
    }
    
    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    }

    public void setState(SwerveModuleState state) {
      if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
      }
      state = SwerveModuleState.optimize(state, getState().angle);
      // driveMotor.set(state.speedMetersPerSecond / MAX_PHYSICAL_SPEED_METERS_PER_SEC);
      // turnMotor.set(ControlMode.PercentOutput, turnPIDController.calculate(getTurnPosition(), state.angle.getRadians()));
      SmartDashboard.putNumber("angle encoder ["+absoluteEncoder.getChannel()+"]", getTurnPosition());
      SmartDashboard.putString("Swerve["+absoluteEncoder.getChannel()+"] state", state.toString());
    }

    public void stop() {
      driveMotor.set(0);
      turnMotor.set(ControlMode.PercentOutput, 0);
    }

    public void brakeMode() {
      driveMotor.setIdleMode(IdleMode.kBrake);
      turnMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void coastMode() {
      driveMotor.setIdleMode(IdleMode.kCoast);
      turnMotor.setNeutralMode(NeutralMode.Coast);
    }

}
