package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
    public static final Integer i_rollers = 17;
    public static final Integer s_feeder = 13;
    public static final Integer s_shooter = 12;
    public static final Integer s_follower = 14;
    public static final Integer h_upperFeed = 10; // todo Change
    public static final Integer h_lowerFeed = 11;
  }

  /*
   * Game element poses relative to blue origin.
   */
  public static class Poses {
    public static final Pose2d tower =
        new Pose2d(Meters.of(1.868), Meters.of(4.162), new Rotation2d());
    public static final Pose2d hub =
        new Pose2d(Meters.of(4.611), Meters.of(4.021), new Rotation2d());
  }

  // Derived from relationship between distance (ft) and rotation (RPM).
  public double base = 1172.95283;
  public double exponential = 0.583488;

  public AngularVelocity regress(Distance distance) {
    return RPM.of(base * Math.pow(distance.in(Feet), exponential));
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
