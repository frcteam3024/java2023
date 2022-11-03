package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** FILE DESCRIPTION: 
 *  This class defines various methods (or "functions") that act on components
 *  of the drive train. Keep in mind these definitions do not do anything on 
 *  their own, they will need to be called from elsewhere
 */

public class DriveSubsystem extends SubsystemBase {
  
  private final TalonSRX motorLeft1 = new TalonSRX(DriveConstants.MOTOR_LEFT_1_ID);
  /** [TODO] declare motorLeft2, motorRight1, and motorRight2 in similar fashion.
   * 
   *  Each motor is a new instance of a class "TalonSRX" (the motorcontrollers we
   *  use). They take the ID of each motor as an input. Instead of passing the number
   *  directly in, it is better practice to place the ID number in the Constants file
   *  to make the code more organized. Make all these constants zero for now.
   * 
   *  Thses motor variables are called "instance variables" of the class DriveSubsystem
   *  and can be accessed from anywhere in this file
   */

  public DriveSubsystem() {
    // Creates a new DriveSubsystem.
  }

  public void brakeMode() {
    motorLeft1.setNeutralMode(NeutralMode.Brake);
    // [TODO] set remaining motors to brake mode
  }

  public void coastMode() {
    // [TODO] set all motors to coast mode
  }

  public void setLeftMotorOutputs(double speed) {
    // [TODO] set left motor outputs to "speed" using .set()
    // hint: use .set() like you used .setNeutralMode above
  }

  // [TODO] create a method to set right motor outputs to an inputted "speed"

}
