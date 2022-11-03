package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/** FILE DESCRIPTION:
 *  This class actually calls the functions defined in DriveSubsystem, detailing
 *  how they will be used with the joystick inputs provided by RobotContainer.
 */

public class TankDriveCommand extends CommandBase {

  /** [TODO] create instance variable driveSubsystem of type DriveSubsystem and 
   *  speedLeftFunction & speedRightFunction of type Supplier<Double> to store
   *  the objects you just passed here from the RobotContainer
   */

public TankDriveCommand(DriveSubsystem driveSubsystem,
      Supplier<Double> speedLeftFunction, Supplier<Double> speedRightFunction) {
    this.driveSubsystem = driveSubsystem;
    /** [TODO] use keyword "this" to set the instance variables speedLeftFunction 
     *  and speedRightFunction equal to the values of this class equal to the 
     *  lambda functions from RobotContainer.
     * 
     *  Note: This is essentially the same as what you did on codecademy, but using
     *  "this" allows the stored variable's name to be equal to the name of the 
     *  target variable, which is just more convenient.
     */
    addRequirements(driveSubsystem);
  }

  // Called once the command is initialized
  @Override
  public void initialize() {
    // Sets the motors of the drive train to brake mode
    driveSubsystem.brakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = speedLeftFunction.get();
    /** [TODO] get the current y-value of the right joystick and store it in variable
     *  rightSpeed of type double
     */

    /** [TODO] multiply leftSpeed and rightSpeed by one half and store the results
     *  in two doubles leftMotorOutputs and rightMotorOutputs
     */
    
    /** [TODO] use the setLeftMotorOutputs() and setRightMotorOutputs() methods from
     *  the drive subsystem to set the outputs of the left and right motors to the
     *  values you just computed
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // [TODO] set motor outputs on both sides to zero and put the motors in coast mode
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
