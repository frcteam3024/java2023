// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogEncoder;

/** Add your docs here. */
public class DriveModule {

    public String location;     // FL,FR,BL,BR (for use in print labels)
    public int index;           // 0,1,2,3 (for use in for loops)

    public CANSparkMax speedMotor;       // drive module
    public TalonSRX angleMotor;          // motors and
    public AnalogEncoder angleEncoder;   // angle encoder

    public boolean flipped;          // should speed be inverted?
    //public double swerveOffset;

    //public double currentAngle;      // for easy
    //public double speedMotorOutput;  // access by
    //public double angleMotorOutput;  // print functions

    // TODO: getters/setters

    public DriveModule(String location, int index) {
        this.location = location;
        this.index = index;
        flipped = false;

        speedMotor = new CANSparkMax(Constants.SPEED_MOTOR_IDS[index], CANSparkMaxLowLevel.MotorType.kBrushless);
        angleMotor = new TalonSRX(Constants.ANGLE_MOTOR_IDS[index]);
        angleEncoder = new AnalogEncoder(Constants.ENCODER_IDS[index]);
    }

    public double getAngleEncoder() {
        double rawEncoder = angleEncoder.getAbsolutePosition();
        double degEncoder = rawEncoder * 360.0;
        double zeroedEncoder = degEncoder - Constants.ENCODER_ZERO_OFFSETS[index];
        double finalEncoder = Robot.driveTrain.standardizeAngle(zeroedEncoder);

        return finalEncoder;
    }
}
