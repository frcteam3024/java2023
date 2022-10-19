package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveTrain;

  public TankDrive(DriveTrain subsystem) {
    driveTrain = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the motors to "coast mode"
    driveTrain.coastMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /** [TODO] use the getCopilotRawAxis method from the RobotContainer file
        to get the inputs from the right and left copilot axes and store those
        values in two different variables. 

        getCopilotRawAxis() takes an axis id as an argument. Create a constant
        in the Constants file for now and pass that as the input.
     */
    double driverLeftAxis = Robot.robotContainer. // fill in the rest of the line
    
    /** [TODO] take the axis values and multiply them by half. Store these as
     *  the left and right target speeds */
    double leftTargetSpeed = // [...]

    /** [TODO] use the setLeftMotors() and setRightMotors() from DriveTrain
     *  to set the motors speed equal to the target speeds of each side */
    // [...]
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // [TODO] set motor speeds to zero and set motors to brake mode 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
