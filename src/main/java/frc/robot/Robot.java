package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  // public static RobotContainer robotContainer;

  @Override
  public void robotInit() {
    // RobotContainer robotContainer = new RobotContainer();
  } 

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    /** This function is called once each time the robot enters Disabled mode. */
  }

  @Override
  public void disabledPeriodic() {
    /** This function is called periodically while the robot is disabled (?) */
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
    /** This function is called at the start of teleop mode. */
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
    /** This function is called periodically whilst in simulation. */
  }
}
