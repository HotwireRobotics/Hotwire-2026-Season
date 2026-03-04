package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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
    public static final Time kChargeUpTime = Seconds.of(0.2);
    public static final Time kFiringTime = Seconds.of(2.3);
    public static final AngularVelocity kSpeed = RPM.of(2500);
  }

  public static class Intake {
    public static final double kSpeed = 0.7;
  }

  public static class Hopper {
    public static final double kSpeed = 0.6;
  }

  public static class Control {
    public static final PIDConstants translationPID = new PIDConstants(25.0, 0.0, 0.0);
    public static final PIDConstants rotationPID = new PIDConstants(25.0, 0.0, 0.0);
    public static final double ANGLE_KP = rotationPID.kP;
    public static final double ANGLE_KD = rotationPID.kD;
  }

  public static class Indication {
    public static SolidColor LEDColor(int r, int g, int b, int w) {
      return new SolidColor(0, 67).withColor(new RGBWColor(255, 255, 255));
    }

    public static class Autonomous {
      public static final Time[] haptic = {};

      public static final TimeTrigger[] times = {
        new TimeTrigger(
            Seconds.of(0), Seconds.of(10), Constants.Indication.LEDColor(255, 255, 255, 255))
      };
    }

    public static class Teloperated {
      public static final Time[] haptic = {
        Seconds.of(10),
        Seconds.of(25),
        Seconds.of(50),
        Seconds.of(75),
        Seconds.of(100),
        Seconds.of(125),
      };

      public static final TimeTrigger[] times = {
        new TimeTrigger(
            Seconds.of(0), Seconds.of(10), Constants.Indication.LEDColor(255, 255, 255, 255))
      };
    }
  }

  public static class TimeTrigger {

    private Time target;
    private Time magnitude;
    private SolidColor color;

    public TimeTrigger(Time time, Time magnitude, SolidColor color) {
      this.target = time;
      this.magnitude = magnitude;
      this.color = color;
    }

    public boolean isTriggered(Time time) {
      return true;
      // double difference = target.minus(time).in(Seconds);
      // if ((difference < magnitude.in(Seconds)) && (difference >= 0)) return true;
      // return false;
    }

    public SolidColor getColor() {
      return color;
    }
  }

  public static final double lerp = 1.7; // 1.7

  public static class Limelight {
    public static final String[] localization = {
      "limelight-gamma", "limelight-alpha", "limelight-two"
    };
    public static final String[] limelights = {"limelight-gamma", "limelight-alpha"};
    public static final Distance maxDistance = Meters.of(2.8);
  }

  public static final Time autoLength = Seconds.of(20);
  public static final Time teleopLength = Seconds.of(140);

  public static class MotorIDs {
    public static final Integer i_rollers = 17;
    public static final Integer s_feeder = 11;
    public static final Integer s_shooterR = 12;
    public static final Integer s_shooterL = 13;
    public static final Integer h_hopperU = 14;
    public static final Integer i_arm = 15;
  }

  public static final PathConstraints constraints =
      new PathConstraints(4, 4, Units.degreesToRadians(540), Units.degreesToRadians(720));

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

  public static Angle flipAlliance(Angle angle) {
    if (DriverStation.getAlliance().get().equals(Alliance.Red)) {
      return angle.plus(Degrees.of(180));
    }
    return angle;
  }

  public static class Poses {
    // X: 14.916m, Y: 3.875m
    public static final Pose2d tower =
        flipAlliance(new Pose2d(Meters.of(1.5653), Meters.of(4.146), Rotation2d.k180deg));
    public static final Pose2d hub =
        flipAlliance(new Pose2d(Meters.of(4.625594), Meters.of(3.965), Rotation2d.k180deg));
    public static final Pose2d lowerStart =
        flipAlliance(new Pose2d(Meters.of(3.583), Meters.of(1.965326), Rotation2d.k180deg));
  }

  // Derived from relationship between distance (m) and rotation (RPM).
  public static final double base = 1705;
  public static final double exponential = 0.5;

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
