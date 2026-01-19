package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double lerp = 1; // 1.7
  public static final String[] limelights = {"limelight-one", "limelight-two"};

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
