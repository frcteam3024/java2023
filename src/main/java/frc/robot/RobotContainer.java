package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OIConstants;

/** FILE DESCRIPTION:
 *  Since TankDriveCommand does nothing on its own, it must be initialized here.
 *  This class creates an instance of the TankDriveCommand and gives it the inputs
 *  on the joystick, allowing the user to actually control the robot. 
 */

public class RobotContainer {

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  /** [TODO] define an instance variable here:
   *  copilotController, a new instance of the class Joystick.
   *  It will need the joystick's port number, so this will need to be a constant.
   *  Create a class OIConstants within Constants, then add COPILOT_JOYSTICK_PORT there
   */

  public double getCopilotRawAxis(int axis) {
    /** [TODO] The copilotController, a joystick, has a method getRawAxis(). 
     *  It takes axis, an integer identifying the axis ID as input and returns
     *  the value of the inputted axis. Use that value as the reutrn value for
     *  this method
     */
  }

  public RobotContainer() {
  
    driveSubsystem.setDefaultCommand(new TankDriveCommand(
        // [TODO] give the driveSubsystem as an argument to the new command 
              ,
        () -> getCopilotRawAxis(OIConstants.COPILOT_LEFT_AXIS),
        /** [TODO] the above line has a lambda expression to return the current
         *  value of an axis on the joystick.
         *  Add COPILOT_LEFT_AXIS to the OIConstants subclass of Constants, then
         *  crate a lambda exxpression below to return the current value of the 
         *  right axis.
         */
    ));
  }

}
