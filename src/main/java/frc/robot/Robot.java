package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// There's no need to mess with anything here yet.

public class Robot extends TimedRobot {

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

}
