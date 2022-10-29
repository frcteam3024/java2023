// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.AnalogEncoder;

/** Add your docs here. */
public class SwerveModule {

    private String location;              // FL,FR,BL,BR (for use in print labels)
    private int index;                    // 0,1,2,3 (for use in for loops)
    private boolean isFlipped;            // should speed be inverted?
    private double swerveOffset;          // degrees offset from 0

    private CANSparkMax speedMotor;       // drive module
    private TalonSRX angleMotor;          // motors and
    private AnalogEncoder angleEncoder;   // angle encoder

    public SwerveModule(String location, int index) {
        this.setLocation(location);
        this.setIndex(index);
        this.setFlip(false);
        this.setSwerveOffset(Constants.ENCODER_ZERO_OFFSETS[index]);

        speedMotor = new CANSparkMax(Constants.SPEED_MOTOR_IDS[index], CANSparkMaxLowLevel.MotorType.kBrushless);
        angleMotor = new TalonSRX(Constants.ANGLE_MOTOR_IDS[index]);
        angleEncoder = new AnalogEncoder(Constants.ENCODER_IDS[index]);
    }

    public String getLocation() {
      return location;
    }
    public void setLocation(String location) {
      this.location = location;
    }

    public int getIndex() {
      return index;
    }
    public void setIndex(int index) {
      this.index = index;
    }

    public boolean getFlip() {
      return isFlipped;
    }
    public void setFlip(boolean isFlipped) {
      this.isFlipped = isFlipped;
    }

    public double getSwerveOffset() {
      return swerveOffset;
    }
    public void setSwerveOffset(double swerveOffset) {
      this.swerveOffset = swerveOffset;
    }

    public double getAngleEncoder() {
        double rawEncoder = angleEncoder.getAbsolutePosition();
        double degEncoder = rawEncoder * 360.0;
        double zeroedEncoder = degEncoder - this.swerveOffset;
        return Robot.driveTrain.standardizeAngle(zeroedEncoder);
    }

    public void brakeMode() {
      speedMotor.setIdleMode(IdleMode.kBrake);
      angleMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void coastMode() {
      speedMotor.setIdleMode(IdleMode.kCoast);
      angleMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setAngleMotorOutput(double output) {
      angleMotor.set(ControlMode.PercentOutput, output);
    }

    public void setSpeedMotorOutput(double output) {
      speedMotor.set(output);
    }

    public void resetOffsets() {
      angleEncoder.reset();
    }
}
