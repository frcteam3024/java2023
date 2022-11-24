package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.TankDriveCmd;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.OIConstants.*;

public class RobotContainer {
  
  // private final IntakeSystem intakeSystem = new IntakeSystem();
  private final DriveSubsystem driveTrain = new DriveSubsystem();
  private final Joystick copilotController = new Joystick(COPILOT_JOYSTICK_PORT);

  public RobotContainer() {

    // intakeSystem.setDefaultCommand(new SpinIntake(
    //   intakeSystem,
    //   () -> getCopilotRawAxis(COPILOT_LEFT_AXIS)
    //   ));

    driveTrain.setDefaultCommand(new TankDriveCmd(
      driveTrain,
      () -> getCopilotRawAxis(COPILOT_LEFT_AXIS),
      () -> getCopilotRawAxis(COPILOT_RIGHT_AXIS)
    ));

    configureButtonBindings();

  }
  
  public double getCopilotRawAxis(int axis) {
    return copilotController.getRawAxis(axis);
  }

  private void configureButtonBindings() { /* TODO document why this method is empty */ }

}