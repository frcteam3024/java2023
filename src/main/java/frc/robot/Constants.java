// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean DUBUG_MODE                  = true;

    // TODO: better method to reset offsets
    public static final double[] SWERVE_OFFSETS = {    // temporary
        0.111,  //FL offset
        0.504,  //FR offset
        0.393,  //BL offset
        0.794,  //BR offset
    };
    
    public static final int FL_MODULE_ID                    = 0;
    public static final int FR_MODULE_ID                    = 1;
    public static final int BL_MODULE_ID                    = 2;
    public static final int BR_MODULE_ID                    = 3;

    public static final int BR_ENCODER_ID                   = 0;
    public static final int BL_ENCODER_ID                   = 1;
    public static final int FR_ENCODER_ID                   = 2;
    public static final int FL_ENCODER_ID                   = 3;

    public static final int BR_ANGLE_MOTOR_ID               = 3;
    public static final int BL_ANGLE_MOTOR_ID               = 4;
    public static final int FR_ANGLE_MOTOR_ID               = 5;
    public static final int FL_ANGLE_MOTOR_ID               = 6;
    
    public static final int BR_SPEED_MOTOR_ID               = 11;
    public static final int BL_SPEED_MOTOR_ID               = 12;
    public static final int FR_SPEED_MOTOR_ID               = 13;
    public static final int FL_SPEED_MOTOR_ID               = 14;

    public static final int INTAKE_MOTOR_ID                 = 8;

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

    public static final int DRIVE_JOYSTICK_PORT             = 0;
    public static final int COPILOT_JOYSTICK_PORT           = 1;
              
    public static final int DRIVE_X_AXIS                    = 0;
    public static final int DRIVE_Y_AXIS                    = 1;
    public static final int DRIVE_ROTATE                    = 2;
    public static final int DRIVE_SLIDER                    = 3;

    public static final double DRIVE_X_AXIS_THRESHOLD       = 0.05;
    public static final double DRIVE_Y_AXIS_THRESHOLD       = 0.05;
    public static final double DRIVE_ROTATE_THRESHOLD       = 0.2;
    public static final double DRIVE_SLIDER_THRESHOLD       = 0.05;

    public static final double[] DRIVER_AXIS_THRESHOLDS = {
        DRIVE_X_AXIS_THRESHOLD,
        DRIVE_Y_AXIS_THRESHOLD,
        DRIVE_ROTATE_THRESHOLD,
        DRIVE_SLIDER_THRESHOLD
    };

    public static final int COPILOT_LEFT_STICK_X            = 0;
    public static final int COPILOT_LEFT_STICK_Y            = 1;
    public static final int COPILOT_RIGHT_STICK_X           = 2;
    public static final int COPILOT_RIGHT_STICK_Y           = 3;
    public static final int COPILOT_X                       = 0;
    public static final int COPILOT_A                       = 1;
    public static final int COPILOT_B                       = 2;
    public static final int COPILOT_Y                       = 3;
    public static final int COPILOT_LB                      = 4;
    public static final int COPILOT_LT                      = 6;
    public static final int COPILOT_RT                      = 7;
    public static final int COPILOT_START                   = 9;
    public static final int COPILOT_LEFT_STICK_PRESS        = 10;
    public static final int COPILOT_RIGHT_STICK_PRESS       = 11;
    /**
    BROKEN:
    public static final int COPILOT_RB                      = 5;
    public static final int COPILOT_BACK                    = 8;
    */

    // inverse tangent of frame dimensions, etc.
    public static double FL_ROTATION_DIRECTION              = 49.6;
    public static double FR_ROTATION_DIRECTION              = -49.6;
    public static double BL_ROTATION_DIRECTION              = -49.6;
    public static double BR_ROTATION_DIRECTION              = 49.6;

    public static final double[] ROTATION_DIRECTIONS = {
        FL_ROTATION_DIRECTION,
        FR_ROTATION_DIRECTION,
        BL_ROTATION_DIRECTION,
        BR_ROTATION_DIRECTION
    };

    public static double kP                                = 0.2;
    public static double kI                                = 0;
    public static double kD                                = 0;

    public static double ANGLE_FLIP_THRESHOLD              = 120;
}
