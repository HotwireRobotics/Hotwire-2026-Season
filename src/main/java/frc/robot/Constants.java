package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class Mathematics {
    public static final double TAU = 6.283185307179586;
  }

  public static class Joysticks {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);
  }

  public static class Shooter {
    public static final Time kChargeUpTime = Seconds.of(2);
  }

  public static class Control {
    public static final PIDConstants translationPID = new PIDConstants(15.0, 0.0, 0.0);
    public static final PIDConstants rotationPID = new PIDConstants(10.0, 0.0, 0.6);
    public static final double ANGLE_KP = rotationPID.kP;
    public static final double ANGLE_KD = rotationPID.kD;
  }

  public static final double lerp = 1; // 1.7
  public static final String[] limelights = {"limelight-one"};
  public static final Time[] autoTimes = {Seconds.of(8)};
  public static final Time[] teleopTimes = {Seconds.of(12)};
  public static final Time autoLength = Seconds.of(20);
  public static final Time teleopLength = Seconds.of(140);

  public static class MotorIDs {
    public static final Integer i_rollers = 13;
    public static final Integer i_follower = 9;
    public static final Integer s_feederR = 11;
    public static final Integer s_shooterR = 8;
    public static final Integer s_shooterL = 15;
    public static final Integer h_upperFeed = 14;
    public static final Integer h_lowerFeed = 12;
  }

  public static final PathConstraints constraints =
      new PathConstraints(2.9, 2.9, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /*
   * Game element poses relative to blue origin.
   */
  public static final Translation2d middle = new Translation2d(Meters.of(8.27), Meters.of(4.01));

  public static Pose2d flipAlliance(Pose2d pose) {
    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      return pose.rotateAround(middle, Rotation2d.k180deg);
    }
    return pose;
  }

  public static class Poses {
    // X: 14.916m, Y: 3.875m
    public static final Pose2d tower =
        flipAlliance(new Pose2d(Meters.of(1.5653), Meters.of(4.146), new Rotation2d()));
    public static final Pose2d hub =
        flipAlliance(new Pose2d(Meters.of(4.611), Meters.of(4.021), new Rotation2d()));
  }

  // Derived from relationship between distance (m) and rotation (RPM).
  public static double base = 1400.9197;
  public static double exponential = 0.450548;

  public static AngularVelocity regress(Distance distance) {
    return RPM.of(base * Math.pow(distance.in(Meters), exponential));
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
