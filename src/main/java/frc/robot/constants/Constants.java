package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.RGBWColor;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Time tracking for match.
  public static final Timer timer = new Timer();

  public static class Mathematics {
    public static final double TAU = 6.283185307179586;
  }

  public static class Joysticks {
    // Static joystick instances for driver and operator controllers.
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);
  }

  public static class Shooter {
    // Static time intervals for firing states.
    public static final Time kChargeUpTime = Seconds.of(0.1);
    public static final Time kFiringTime = Seconds.of(8.2);
    public static final Time k35Time = Seconds.of(3);
    public static final Time k60Time = Seconds.of(3);
    public static final Time kDebounce = Seconds.of(0.1);

    // Static target velocities and tolerances.
    public static final AngularVelocity kSpeed = RPM.of(2300);
    public static final AngularVelocity kVelocityTolerance = RotationsPerSecond.of(7);
    public static final AngularVelocity kZero = RPM.of(0);

    // Drivetrain alignment error tolerance.
    public static final Angle kAlignmentError = Degrees.of(4);

    // Current limits for shooter motors.
    public static final Current kCurrentLimit = Amps.of(80);
  }

  public static class Intake {
    // Static speed for intake rollers.
    public static final double kSpeed = 0.8;
    // Arm oscillation frequency.
    public static final Frequency kOscillationFrequency = Hertz.of(2.62);
  }

  public static class Hopper {
    // Static speed for hopper rollers.
    public static final double kSpeed = 0.8;
  }

  public static class Control {
    public static final PIDConstants translationPID = new PIDConstants(25.0, 0.0, 0.0);
    public static final PIDConstants rotationPID = new PIDConstants(13.0, 0.0, 0.0);
    public static final double ANGLE_KP = rotationPID.kP;
    public static final double ANGLE_KD = rotationPID.kD;
  }

  public static class Tempo {
    // Mutable time tracking fields.
    private static double timerOffset = 0;
    private static Time time = Seconds.of(0);

    /** Start the timer from zero. */
    public static void startTime() {
      timer.reset();
      timer.start();
    }

    /**
     * Start the timer with an offset, used for simulating delayed starts and autonomous periods.
     *
     * @param offset
     */
    public static void startTime(Time offset) {
      startTime();
      timerOffset = offset.in(Seconds);
    }

    /** Update time measurement. */
    public static Time tick() {
      time = Seconds.of(timer.get() + timerOffset);
      Logger.recordOutput("Time", time.in(Seconds));

      return time;
    }

    /** Get time measurement. */
    public static Time getTime() {
      return time;
    }

    /**
     * Identify if the specified time has elapsed.
     *
     * @param target
     */
    public static boolean isElapsed(Time target) {
      return time.gte(target);
    }

    /**
     * Identify if the current time is within the specified range.
     *
     * @param start
     * @param end
     */
    public static boolean isRange(Time start, Time end) {
      return time.gte(start) && time.lte(end);
    }
  }

  public static class Indication {
    /**
     * Creates a solid color control request for the CANdle.
     *
     * @param r Red
     * @param g Green
     * @param b Blue
     */
    public static SolidColor LEDColor(int r, int g, int b) {
      return new SolidColor(0, 67).withColor(new RGBWColor(r, g, b));
    }

    // /**
    //  * Creates an alternating pattern of x lights on, x lights off; this is offset by p
    //  *
    //  * @param r Red
    //  * @param g Green
    //  * @param b Blue
    //  */
    // public static SolidColor CHASEColor(int r, int g, int b) {
    //   return new Color
    // }

    /**
     * Determine if the robot is on track for an autonomous victory, based on the first character of
     * the game-specific message and the alliance color.
     */
    public static boolean autonomousVictory() {
      // Read driverstation.
      String gameData = DriverStation.getGameSpecificMessage();
      Boolean allianceIsRed = getAlliance().equals(Alliance.Red);

      // Switch based on game data.
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

    /**
     * Get the alliance color for this robot.
     *
     * @return
     */
    public static Alliance getAlliance() {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance == null) return Alliance.Red;
      return alliance.get();
    }

    /** Control haptic indicators based on time remaining in the match. */
    public static void updateHaptics() {
      Time time = Tempo.getTime();

      // Control haptic indicators.
      Boolean rumble = false;

      for (Time target : Constants.Indication.transitions) {
        double difference = target.minus(time).in(Seconds);
        if ((Math.abs(difference) < 1) && (difference < 0)) {
          rumble = true;
        }
      }

      // Apply haptics.
      Constants.Joysticks.driver.setRumble(RumbleType.kLeftRumble, rumble ? 1 : 0);
    }

    public static enum Period {
      AUTONOMOUS,
      TRANSITION,
      PRIMARY,
      SECONDARY,
      TERTIARY,
      QUATERNARY,
      ENDGAME
    }

    /**
     * Identify the current period of the match based on the specified time.
     *
     * @param t
     */
    public static Period fromTime(Time t) {
      if (t.lte(Length.autonomous)) return Period.AUTONOMOUS;
      if (t.lte(Length.autonomous.plus(Length.transition))) return Period.TRANSITION;
      if (t.lte(Length.autonomous.plus(Length.phase).plus(Length.transition)))
        return Period.PRIMARY;
      if (t.lte(Length.autonomous.plus(Length.phase.times(2)).plus(Length.transition)))
        return Period.SECONDARY;
      if (t.lte(Length.autonomous.plus(Length.phase.times(3)).plus(Length.transition)))
        return Period.TERTIARY;
      if (t.lte(Length.autonomous.plus(Length.phase.times(4)).plus(Length.transition)))
        return Period.QUATERNARY;
      return Period.ENDGAME;
    }

    /** Identify if the robot is within an active period. */
    public static boolean isActive() {
      Boolean victoryAuto = autonomousVictory();
      Period period = fromTime(Tempo.getTime());
      switch (period) {
        case AUTONOMOUS:
        case ENDGAME:
        case TRANSITION:
          return true;
        case SECONDARY:
        case QUATERNARY:
          return victoryAuto;
        case PRIMARY:
        case TERTIARY:
          return !victoryAuto;
        default:
          return false;
      }
    }

    /** Identify if the specified time is within an active period for the robot. */
    public static boolean isTimeActive(Time t) {
      Boolean victoryAuto = autonomousVictory();
      Period period = fromTime(t);
      switch (period) {
        case AUTONOMOUS:
        case ENDGAME:
        case TRANSITION:
          return true;
        case SECONDARY:
        case QUATERNARY:
          return victoryAuto;
        case PRIMARY:
        case TERTIARY:
          return !victoryAuto;
        default:
          return false;
      }
    }

    /** Identify if the robot is within a warning period before a transition. */
    public static boolean isWaning() {
      return isActive() && !isTimeActive(Tempo.getTime().plus(warning));
    }

    /** Identify if the robot is within a warning period before a transition. */
    public static boolean isWaxing() {
      return !isActive() && isTimeActive(Tempo.getTime().plus(warning));
    }

    /** Identify if the robot should begin shooting. */
    public static boolean isPreping() {
      return !isActive() && isTimeActive(Tempo.getTime().plus(preping));
    }

    public static final Time warning = Seconds.of(7);
    public static final Time preping = Seconds.of(2);

    public static final Time[] transitions = {
      Seconds.of(20), Seconds.of(30), Seconds.of(55),
      Seconds.of(80), Seconds.of(105), Seconds.of(130),
    };
  }

  public static final double lerp = 1.7;

  /** Limelight configuration and constants. */
  public static class Limelight {
    public static final String[] localization = {"limelight-gamma", "limelight-alpha"};
    public static final String[] limelights = {"limelight-gamma", "limelight-alpha"};
    public static final Distance maxDistance = Inches.of(100);
  }

  /** Match time periods. */
  public static class Length {
    public static final Time autonomous = Seconds.of(20);
    public static final Time delay = Seconds.of(3);
    public static final Time transition = Seconds.of(10);
    public static final Time phase = Seconds.of(25); // x4
    public static final Time endgame = Seconds.of(30);

    public static final Time teleoperated = Seconds.of(140);
  }

  /**
   * Motor IDs for all devices, organized by subsystem. These should match values in Phoenix Tuner.
   */
  public static class MotorIDs {
    public static final Integer i_rollers = 17;
    public static final Integer s_feeder = 11;
    public static final Integer s_shooterR = 12;
    public static final Integer s_shooterL = 13;
    public static final Integer h_hopper = 14;
    public static final Integer i_wrist = 15;
  }

  public static final PathConstraints constraints =
      new PathConstraints(4, 4, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /*
   * Game element poses relative to blue origin.
   */
  public static final Translation2d middle = new Translation2d(Meters.of(8.27), Meters.of(4.01));

  /**
   * Flip a pose based on alliance color.
   *
   * @param pose
   */
  public static Pose2d allianceRelative(Pose2d pose) {
    if (Constants.Indication.getAlliance().equals(Alliance.Red)) {
      return pose.rotateAround(middle, Rotation2d.k180deg);
    }
    return pose;
  }

  /**
   * Flip an angle based on alliance color.
   *
   * @param angle
   */
  public static Angle allianceRelative(Angle angle) {
    if (Constants.Indication.getAlliance().equals(Alliance.Red)) {
      return angle.plus(Degrees.of(180));
    }
    return angle;
  }

  public static class Poses {
    public static class AllianceRelativePose {
      // Blue-origin relative pose.
      private final Pose2d pose;

      /**
       * Assumes a blue-origin relative pose.
       */
      public AllianceRelativePose(Pose2d pose) {
        this.pose = pose;
      }

      /**
       * Assumes a set-origin relative pose.
       * 
       * @param alliance side of provided pose.
       */
      public AllianceRelativePose(Pose2d pose, Alliance alliance) {
        this(alliance.equals(Alliance.Red) ? pose.rotateAround(middle, Rotation2d.k180deg) : pose);
      }

      /**
       * Exposes relative pose.
       */
      public Pose2d getPose() {
        return allianceRelative(pose);
      }

      /**
       * Expose blue-origin relative pose.
       */
      public Pose2d getBluePose() {
        return getPose();
      }

      /**
       * Expose red-origin relative pose.
       */
      public Pose2d getRedPose() {
        return getPose().rotateAround(middle, Rotation2d.k180deg);
      }
    }
    
    // Define relative poses.
    public static final AllianceRelativePose tower = 
        new AllianceRelativePose(new Pose2d(Meters.of(1.5653), Meters.of(4.146), Rotation2d.k180deg));
    public static final AllianceRelativePose hub = 
        new AllianceRelativePose(new Pose2d(Meters.of(4.625594), Meters.of(3.965), Rotation2d.k180deg));
    public static final AllianceRelativePose lowerStart = 
        new AllianceRelativePose(new Pose2d(Meters.of(3.583), Meters.of(1.965326), Rotation2d.k180deg));
            
  }

  // Derived from relationship between distance (m) and rotation (RPM).
  public static final double base = 1300.92838;
  public static final double exponential = 1.00529;

  /**
   * Calculate shooter velocity from distance using an exponential regression.
   *
   * @param distance Distance to the target.
   * @return Shooter velocity in RPM.
   */
  public static AngularVelocity regress(Distance distance) {
    Logger.recordOutput("Shooter/Distance", distance.in(Inches));
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
