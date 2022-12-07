// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import static java.lang.Math.PI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private Constants() {}

  public static final boolean DUBUG_MODE                  = true;

  /**
  public static final double FL_ENCODER_ZERO_OFFSET       = 39.2;
  public static final double FR_ENCODER_ZERO_OFFSET       = 173.1;
  public static final double BL_ENCODER_ZERO_OFFSET       = 137.9;
  public static final double BR_ENCODER_ZERO_OFFSET       = -73.8;
  public static final double[] ENCODER_ZERO_OFFSETS = {
      FL_ENCODER_ZERO_OFFSET,
      FR_ENCODER_ZERO_OFFSET,
      BL_ENCODER_ZERO_OFFSET,
      BR_ENCODER_ZERO_OFFSET
  };
  
  public static final int FL_SWERVE_MODULE_ID             = 0;
  public static final int FR_SWERVE_MODULE_ID             = 1;
  public static final int BL_SWERVE_MODULE_ID             = 2;
  public static final int BR_SWERVE_MODULE_ID             = 3;

  public static final int BR_ENCODER_ID                   = 0;
  public static final int BL_ENCODER_ID                   = 1;
  public static final int FR_ENCODER_ID                   = 2;
  public static final int FL_ENCODER_ID                   = 3;

  public static final int BR_ANGLE_MOTOR_ID               = 4;
  public static final int BL_ANGLE_MOTOR_ID               = 5;
  public static final int FR_ANGLE_MOTOR_ID               = 6;
  public static final int FL_ANGLE_MOTOR_ID               = 7;
  

  public static final int BR_SPEED_MOTOR_ID               = 11;
  public static final int BL_SPEED_MOTOR_ID               = 12;
  public static final int FR_SPEED_MOTOR_ID               = 13;
  public static final int FL_SPEED_MOTOR_ID               = 14;

  public static final int[] ENCODER_IDS = {
      FL_ENCODER_ID,
      FR_ENCODER_ID,
      BL_ENCODER_ID,
      BR_ENCODER_ID,
  };
  public static final int[] ANGLE_MOTOR_IDS = {
      FL_ANGLE_MOTOR_ID,
      FR_ANGLE_MOTOR_ID,
      BL_ANGLE_MOTOR_ID,
      BR_ANGLE_MOTOR_ID,
  };
  public static final int[] SPEED_MOTOR_IDS = {
      FL_SPEED_MOTOR_ID,
      FR_SPEED_MOTOR_ID,
      BL_SPEED_MOTOR_ID,
      BR_SPEED_MOTOR_ID,
  };


  */
  public static final class ModuleConstants {
    private ModuleConstants() {}

    public static final double WHEEL_DIAMETER                      = .105;
    public static final double DRIVE_MOTOR_GEAR_RATIO              = 1.0/4.0;    // ?? Davis' guess
    public static final double DRIVE_ENCODER_ROTATION_TO_METER     = DRIVE_MOTOR_GEAR_RATIO * PI * WHEEL_DIAMETER;
    public static final double DRIVE_ENCODER_RPM_TO_METERS_PER_SEC = DRIVE_ENCODER_ROTATION_TO_METER / 60.0;
    public static final double KP_TURN                             = 0.15;
    public static final double KI_TURN                             = 0;
    public static final double KD_TURN                             = 0;
  }

  public static final class DriveConstants {
    private DriveConstants() {}

    // distance between left and right wheels (meters)
    public static final double TRACK_WIDTH = Units.inchesToMeters(20);
    // distance between front and back wheels (meters)
    public static final double WHEEL_BASE = Units.inchesToMeters(23.5);
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      // each argument is the location of a swerve module relative to the robot's center
      new Translation2d( WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d( WHEEL_BASE / 2,  TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2,  TRACK_WIDTH / 2)
    );

    public static final int     FL_DRIVE_MOTOR_PORT             = 14;
    public static final int     FL_TURN_MOTOR_PORT              = 7;
    public static final boolean FL_DRIVE_ENCODER_REVERSED       = false;
    public static final boolean FL_TURN_ENCODER_REVERSED        = false;
    public static final int     FL_ABSOLUTE_ENCODER_PORT        = 3;
    public static final double  FL_ABSOLUTE_ENCODER_OFFSET_RAD  = 0.684;
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED    = false;

    public static final int     FR_DRIVE_MOTOR_PORT             = 13;
    public static final int     FR_TURN_MOTOR_PORT              = 6;
    public static final boolean FR_DRIVE_ENCODER_REVERSED       = false;
    public static final boolean FR_TURN_ENCODER_REVERSED        = false;
    public static final int     FR_ABSOLUTE_ENCODER_PORT        = 2;
    public static final double  FR_ABSOLUTE_ENCODER_OFFSET_RAD  = 3.021;
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED    = false;

    public static final int     BL_DRIVE_MOTOR_PORT             = 12;
    public static final int     BL_TURN_MOTOR_PORT              = 5;
    public static final boolean BL_DRIVE_ENCODER_REVERSED       = false;
    public static final boolean BL_TURN_ENCODER_REVERSED        = false;
    public static final int     BL_ABSOLUTE_ENCODER_PORT        = 1;
    public static final double  BL_ABSOLUTE_ENCODER_OFFSET_RAD  = 2.407;
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED    = false;

    public static final int     BR_DRIVE_MOTOR_PORT             = 11;
    public static final int     BR_TURN_MOTOR_PORT              = 4;
    public static final boolean BR_DRIVE_ENCODER_REVERSED       = false;
    public static final boolean BR_TURN_ENCODER_REVERSED        = false;
    public static final int     BR_ABSOLUTE_ENCODER_PORT        = 0;
    public static final double  BR_ABSOLUTE_ENCODER_OFFSET_RAD  = -1.288;
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED    = false;

    public static final double MAX_PHYSICAL_SPEED_METERS_PER_SEC          = .2;            // ? 
    public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SEC        = .2;            // ? 
    //public static final double TELE_DRIVE_MAX_ACCEL_UNITS_PER_SEC         = 1;           // ? 
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RAD_PER_SEC   = 2 * PI;        // ?  
    //public static final double TELE_DRIVE_MAX_ANGULAR_ACCEL_UNITS_PER_SEC = 2 * Math.PI; // ? 
  }
  
  public static final class IntakeConstants {
    private IntakeConstants() {}

    public static final int INTAKE_MOTOR_ID = 8;
    public static final double MAX_INTAKE_SPEED = 1;
  }
  public static final class OIConstants {
    OIConstants() {}

    public static final double AXIS_DEADBAND          = 0.05;
    public static final double TURN_DEADBAND          = 0.20;

    public static final int DRIVE_JOYSTICK_PORT       = 0;
    public static final int COPILOT_JOYSTICK_PORT     = 2;
              
    public static final int DRIVE_X_AXIS              = 0;
    public static final int DRIVE_Y_AXIS              = 1;
    public static final int DRIVE_ROTATE_AXIS         = 2;
    public static final int DRIVE_SLIDER              = 3;

    public static final int COPILOT_X_AXIS_LEFT       = 0;
    public static final int COPILOT_Y_AXIS_LEFT       = 1;
    public static final int COPILOT_X_AXIS_RIGHT      = 2;
    public static final int COPILOT_Y_AXIS_RIGHT      = 3;

    public static final int COPILOT_X                 = 1;
    public static final int COPILOT_A                 = 2;
    public static final int COPILOT_B                 = 3;
    public static final int COPILOT_Y                 = 4;
    public static final int COPILOT_LB                = 5;
    public static final int COPILOT_RB                = 6;
    public static final int COPILOT_LT                = 7;
    public static final int COPILOT_RT                = 8;
    public static final int COPILOT_BACK              = 9;
    public static final int COPILOT_START             = 10;
    public static final int COPILOT_LEFT_STICK_PRESS  = 11;
    public static final int COPILOT_RIGHT_STICK_PRESS = 12;

    public static final int FIELD_ORIENTED_TOGGLE_BUTTON = COPILOT_A;
    public static final int RESET_GYRO_BUTTON = COPILOT_B;
    public static final int SPIN_INTAKE_BUTTON = COPILOT_X;

  }


}
