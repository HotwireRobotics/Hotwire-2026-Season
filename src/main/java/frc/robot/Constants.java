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
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final Timer timer = new Timer();

  public static class Mathematics {
    public static final double TAU = 6.283185307179586;
  }

  public static class Joysticks {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);
  }

  public static class Shooter {
    public static final Time kChargeUpTime = Seconds.of(0.1);
    public static final Time kFiringTime = Seconds.of(5);
    public static final Time kUntilAggitateTime = Seconds.of(2.6);
    public static final AngularVelocity kSpeed = RPM.of(2500);
    public static final Angle kAlignmentError = Degrees.of(4);
  }

  public static class Intake {
    public static final double kSpeed = 0.8;
    public static final Frequency kOccilationFrequency = Hertz.of(2.62);
  }

  public static class Hopper {
    public static final double kSpeed = 0.8;
  }

  public static class Control {
    public static final PIDConstants translationPID = new PIDConstants(25.0, 0.0, 0.0);
    public static final PIDConstants rotationPID = new PIDConstants(13.0, 0.0, 0.0);
    public static final double ANGLE_KP = rotationPID.kP;
    public static final double ANGLE_KD = rotationPID.kD;
  }

  public static void startTime() {
    timer.reset();
    timer.start();
  }

  public static Time getTime() {
    Time t = Seconds.of(DriverStation.getMatchTime());
    Time length =
        (DriverStation.isAutonomous())
            ? Constants.Length.autonomous
            : Constants.Length.teleoperated;

    return (t.isEquivalent(Seconds.of(-1))) ? Seconds.of(timer.get()) : length.minus(t);
  }

  public static class Indication {
    public static SolidColor LEDColor(int r, int g, int b) {
      return new SolidColor(0, 67).withColor(new RGBWColor(r, g, b));
    }

    public static boolean autonomousVictory() {
      String gameData = DriverStation.getGameSpecificMessage();
      Boolean allianceIsRed = DriverStation.getAlliance().get().equals(Alliance.Red);
      if (gameData.length() < 1) return true;
      switch (gameData.charAt(0)) {
        case 'R':
          return (allianceIsRed);
        case 'B':
          return (!allianceIsRed);
        default:
          return true;
      }
    }

    public static boolean visionRegister() {
      for (String limelight : Limelight.localization) {
        return true;
      }
      return false;
    }

    public static boolean isActive() {
      return autonomousVictory();
    }

    public static class Autonomous {
      public static final Time[] haptic = {};
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
    }
  }

  public static final double lerp = 1.7; // 1.7

  public static class Limelight {
    public static final String[] localization = {
      "limelight-gamma", "limelight-alpha", "limelight-two"
    };
    public static final String[] limelights = {"limelight-gamma", "limelight-alpha"};
    public static final Distance maxDistance = Inches.of(191);
  }

  public static class Length {
    public static final Time autonomous = Seconds.of(20);
    public static final Time transition = Seconds.of(10);
    public static final Time period = Seconds.of(25); // x4
    public static final Time endgame = Seconds.of(30);

    public static final Time teleoperated = Seconds.of(140);
  }

  public static class MotorIDs {
    public static final Integer i_rollers = 17;
    public static final Integer s_feeder = 11;
    public static final Integer s_shooterR = 12;
    public static final Integer s_shooterL = 13;
    public static final Integer h_hopper = 14;
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
  public static final double base = 1455.92838;
  public static final double exponential = 1.00529;

  public static AngularVelocity regress(Distance distance) {
    return RPM.of(base * Math.pow(exponential, distance.in(Inches)));
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
