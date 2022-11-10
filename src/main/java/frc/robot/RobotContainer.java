package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OIConstants;

/** FILE DESCRIPTION:
 *  We need to create a command to utilize all the functions we just defined in 
 *  DriveSubsystem, which we will call TankDriveCommand. It won't do anything on
 *  its own, so it must be initialized here. We will create a joystick object and
 *  pass its inputs into the TankDriveCommand, allowing the user to drive the robot
 */

public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  /** [TODO #6] define a joystick instance variable here:
   *  copilotController, a new instance of the class Joystick.
   *  It will need the joystick's port number, so this will need to be a constant.
   *  [TODO #7] Create a class OIConstants within Constants, then add COPILOT_JOYSTICK_PORT there
   */

  public double getCopilotRawAxis(int axis) {
    /** [TODO #8] The copilotController, a joystick, has a method getRawAxis(). 
     *  It takes axis, an integer, identifying the axis ID as input and returns
     *  the value of the inputted axis. Use that value as the reutrn value for
     *  this method
     */
  }

  public RobotContainer() {
  
    driveSubsystem.setDefaultCommand(new TankDriveCommand(
        /** TankDriveCommand isn't defined quite yet, but we'll need to give it
         *  our instance of DriveSubsystem and two supplier functions to
         *  give the values of the joystick axes.
         * 
         *  [TODO #9] give driveSubsystem as the first argument
         *  (followed by a comma to separate it from the next arguments)
         */

        () -> getCopilotRawAxis(OIConstants.COPILOT_LEFT_AXIS),
        /** [TODO #10] the above line has a lambda expression to return the current
         *  value of the left axis on the joystick. Add COPILOT_LEFT_AXIS to the
         *  OIConstants subclass of Constants, then create a lambda expression below
         *  to return the current value of the right axis.
         */
    ));
  }

}
