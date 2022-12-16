package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.TankDriveCmd;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final Joystick copilotController = new Joystick(COPILOT_JOYSTICK_PORT);
  private final Joystick driverController = new Joystick(DRIVER_JOYSTICK_PORT);

  public RobotContainer() {

    // driveSubsystem.setDefaultCommand(new TankDriveCmd(
    //   driveSubsystem,
    //   () -> getCopilotRawAxis(COPILOT_LEFT_AXIS),
    //   () -> getCopilotRawAxis(COPILOT_RIGHT_AXIS)
    // ));

    driveSubsystem.setDefaultCommand(
      new TankDriveCmd(
        driveSubsystem,
        () -> getCopilotRawAxis(COPILOT_LEFT_Y_AXIS),
        () -> getCopilotRawAxis(COPILOT_RIGHT_Y_AXIS)
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

  private Command configureButtonBindings() {
    return null;
  }
}