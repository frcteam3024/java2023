package frc.robot;

public final class Constants {
  private Constants() {}

  public static final class DriveConstants {
    private DriveConstants() {}
    
    public static final int MOTOR_LEFT_1_ID  = 6;
    public static final int MOTOR_LEFT_2_ID  = 4;
    public static final int MOTOR_RIGHT_1_ID = 5;
    public static final int MOTOR_RIGHT_2_ID = 3;
  }

  public static final class OIConstants {
    private OIConstants() {}

    public static final double MIN_MOTOR_OUTPUT = 0.05;
    public static final int COPILOT_JOYSTICK_PORT = 2;
    public static final int COPILOT_LEFT_Y_AXIS = 1;
    public static final int COPILOT_RIGHT_Y_AXIS = 3;

    public static final int DRIVER_JOYSTICK_PORT = 0;
    public static final int DRIVER_X_AXIS = 0;
    public static final int DRIVER_Y_AXIS = 1;
  }

}
