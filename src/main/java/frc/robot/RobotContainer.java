package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ArcadeDriveCmd;
// import frc.robot.commands.TankDriveCmd;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  
  private final DriveSubsystem driveTrain = new DriveSubsystem();
  private final Joystick copilotController = new Joystick(COPILOT_JOYSTICK_PORT);
  private final Joystick driverController = new Joystick(DRIVER_JOYSTICK_PORT);

  public RobotContainer() {

    // driveTrain.setDefaultCommand(new TankDriveCmd(
    //   driveTrain,
    //   () -> getCopilotRawAxis(COPILOT_LEFT_AXIS),
    //   () -> getCopilotRawAxis(COPILOT_RIGHT_AXIS)
    // ));

    driveTrain.setDefaultCommand(
      new ArcadeDriveCmd(
        driveTrain,
        () -> -getCopilotRawAxis(DRIVER_Y_AXIS),
        () -> getCopilotRawAxis(DRIVER_X_AXIS)
      )
    );

    configureButtonBindings();

  }
  
  public double getCopilotRawAxis(int axis) {
    return copilotController.getRawAxis(axis);
  }

  public double getDriverRawAxis(int axis) {
    return driverController.getRawAxis(axis);
  }

  private void configureButtonBindings() { /* TODO document why this method is empty */ }

}