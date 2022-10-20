package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;

public class Robot extends TimedRobot {
  private DriveTrain driveTrain;
  public static RobotContainer robotContainer;

  @Override
  public void robotInit() {
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */

    robotContainer = new RobotContainer();
    // [TODO] initialize driveTrain as an instance of DriveTrain in a similar fashion
  }

  @Override
  public void robotPeriodic() {
    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    /** This function is called once each time the robot enters Disabled mode. */
  }

  @Override
  public void disabledPeriodic() {
    /** This function is called periodically while the robot is disabled */
  }

  @Override
  public void autonomousInit() {
    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  }

  @Override
  public void autonomousPeriodic() {
    /** This function is called periodically during autonomous. */
  }

  @Override
  public void teleopInit() {
    /** [TODO] initialize a new Command called tankDrive as a new instance of TankDrive.
     *  it will require a subsystem as an argument, so pass in driveTrain that we defined above
     *  Finally, schedule tankDrive with the .schedule() method. This step is what takes all
     *  the code you've written and actually runs it on the robot */

  }

  @Override
  public void teleopPeriodic() {
    /** This function is called periodically during operator control. */
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    /** This function is called periodically during test mode. */
  }

  @Override
  public void simulationInit() {
    /** This function is called once when the robot is first started up. */
  }

  @Override
  public void simulationPeriodic() {
    /** This function is called periodically while in simulation. */
  }
}
