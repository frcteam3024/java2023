package frc.robot;

public final class Constants {
  private Constants() {}

  public static final class DriveConstants {
    private DriveConstants() {}
    
    public static final int MOTOR_LEFT_1_ID = 0;
    public static final int MOTOR_LEFT_2_ID = 6;
    public static final int MOTOR_RIGHT_1_ID = 1;
    public static final int MOTOR_RIGHT_2_ID = 4;
  }

  public static final class OIConstants {
    private OIConstants() {}

    public static final int COPILOT_JOYSTICK_PORT = 2;
    public static final int COPILOT_LEFT_AXIS = 1;
    public static final int COPILOT_RIGHT_AXIS = 3;
    public static final int INTAKE_MOTOR_ID = 8;
  }

}
