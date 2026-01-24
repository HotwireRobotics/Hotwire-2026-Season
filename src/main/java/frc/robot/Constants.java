package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class Joysticks {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);
  }

  public static final double lerp = 1; // 1.7
  public static final String[] limelights = {"limelight-one", "limelight-two"};
  public static final Time[] autoTimes = {Seconds.of(8)};
  public static final Time[] teleopTimes = {Seconds.of(12)};
  public static final Time autoLength = Seconds.of(20);
  public static final Time teleopLength = Seconds.of(140);

  public static class MotorIDs {
    public static final Integer i_rollers = 10;
    public static final Integer i_follower = 11;
    public static final Integer s_feeder = 13;
    public static final Integer s_shooter = 12;
    public static final Integer s_follower = 14;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
