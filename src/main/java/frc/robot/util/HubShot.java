package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleUnaryOperator;

/**
 * Aim and flywheel speed for shooting while the chassis is moving.
 *
 * <p>A stopped robot gets the distance regression unchanged. A moving robot subtracts its field
 * velocity from the ball's desired field velocity, which changes both the heading and the RPM.
 * Every returned number is finite, and the heading offset and RPM scale are clamped.
 */
public final class HubShot {

  /** Ferry RPM used when the inputs cannot produce a shot. */
  public static final double FALLBACK_RPM = 2400.0;

  private HubShot() {}

  /** One solved shot. Every numeric field is finite. */
  public static final class Solution {
    /** Projected chassis translation the shot was solved from. */
    public final Translation2d pose;

    public final Rotation2d aim;
    public final double distanceMeters;
    public final double rpm;
    public final double stationaryRpm;

    /** Heading offset from the raw hub bearing, radians, after clamping. */
    public final double leadRadians;

    /** Chassis speed to the left of the hub ray, m/s. */
    public final double perpMetersPerSecond;

    /** Chassis speed toward the hub, m/s. Positive means closing. */
    public final double radialMetersPerSecond;

    /** False when the inputs were unusable and this is a safe stand-in. */
    public final boolean live;

    /**
     * @param pose projected translation
     * @param aim field heading to hold
     * @param distanceMeters clamped hub distance
     * @param rpm flywheel setpoint after velocity scaling
     * @param stationaryRpm regression RPM before velocity scaling
     * @param leadRadians clamped offset from the raw bearing
     * @param perpMetersPerSecond sideways speed, positive to the left of the ray
     * @param radialMetersPerSecond speed toward the hub
     * @param live true when this was solved from finite inputs
     */
    public Solution(
        Translation2d pose,
        Rotation2d aim,
        double distanceMeters,
        double rpm,
        double stationaryRpm,
        double leadRadians,
        double perpMetersPerSecond,
        double radialMetersPerSecond,
        boolean live) {
      this.pose = pose;
      this.aim = aim;
      this.distanceMeters = distanceMeters;
      this.rpm = rpm;
      this.stationaryRpm = stationaryRpm;
      this.leadRadians = leadRadians;
      this.perpMetersPerSecond = perpMetersPerSecond;
      this.radialMetersPerSecond = radialMetersPerSecond;
      this.live = live;
    }
  }

  /**
   * Tunables and measurements for one solve. Defaults match {@code Constants.Shooter}. The robot
   * overwrites the tunables from Constants and the dashboard before each call.
   */
  public static final class Input {
    public Translation2d robot = Translation2d.kZero;
    public Translation2d hub = new Translation2d(4.0, 0.0);
    public double vxMetersPerSecond;
    public double vyMetersPerSecond;
    public double lookaheadSeconds = 0.10;
    public double metersPerSecondPerRpm = 0.005;
    public double leadGainRadiansPerMps = 0.0;
    public double minScale = 0.70;
    public double maxScale = 1.40;
    public double maxLeadRadians = Math.toRadians(25.0);
    public double maxFieldSpeed = 5.5;
    public double minExitMetersPerSecond = 4.0;
    public double minDistanceMeters = 0.30;
    public double maxDistanceMeters = 8.0;
    public double maxRpm = 5500.0;
    public double regressionBase = 1350.92838;
    public double regressionExp = 1.00529;

    /** When set, replaces the built-in distance regression. */
    public DoubleUnaryOperator rpmForDistance;
  }

  /** True when {@code value} is neither NaN nor infinite. */
  public static boolean isFinite(double value) {
    return Double.isFinite(value);
  }

  /** True when both components are finite. */
  public static boolean isFinite(Translation2d translation) {
    return translation != null && isFinite(translation.getX()) && isFinite(translation.getY());
  }

  /** True when translation and heading are finite. */
  public static boolean isFinite(Pose2d pose) {
    return pose != null
        && isFinite(pose.getX())
        && isFinite(pose.getY())
        && pose.getRotation() != null
        && isFinite(pose.getRotation().getRadians());
  }

  /** True when every chassis-speed component is finite. */
  public static boolean isFinite(ChassisSpeeds speeds) {
    return speeds != null
        && isFinite(speeds.vxMetersPerSecond)
        && isFinite(speeds.vyMetersPerSecond)
        && isFinite(speeds.omegaRadiansPerSecond);
  }

  /** A finite shot used when the pose or the hub cannot be read. */
  public static Solution fallback() {
    return new Solution(
        Translation2d.kZero,
        Rotation2d.kZero,
        0.30,
        FALLBACK_RPM,
        FALLBACK_RPM,
        0.0,
        0.0,
        0.0,
        false);
  }

  /**
   * Solve aim and RPM. Never throws. Non-finite velocity is treated as stopped, so a bad speed
   * reading falls back to the stationary regression instead of yanking the heading.
   *
   * @param in measurements and tunables, or null
   */
  public static Solution solve(Input in) {
    if (in == null || !isFinite(in.robot) || !isFinite(in.hub)) {
      return fallback();
    }

    double maxSpeed = positive(in.maxFieldSpeed, 5.5);
    double vx = MathUtil.clamp(finiteOrZero(in.vxMetersPerSecond), -maxSpeed, maxSpeed);
    double vy = MathUtil.clamp(finiteOrZero(in.vyMetersPerSecond), -maxSpeed, maxSpeed);
    double lookahead = MathUtil.clamp(finiteOrZero(in.lookaheadSeconds), 0.0, 0.40);

    Translation2d future = in.robot.plus(new Translation2d(vx, vy).times(lookahead));
    if (!isFinite(future)) {
      future = in.robot;
    }

    Translation2d toHub = in.hub.minus(future);
    if (!isFinite(toHub)) {
      return fallback();
    }
    double distance = toHub.getNorm();
    if (!isFinite(distance)) {
      return fallback();
    }

    double minDistance = positive(in.minDistanceMeters, 0.30);
    double maxDistance = Math.max(minDistance, positive(in.maxDistanceMeters, 8.0));
    double clampedDistance = MathUtil.clamp(Math.max(distance, 0.0), minDistance, maxDistance);

    // Sitting on the hub has no direction. Hold a zero heading and the minimum distance.
    Rotation2d bearing = distance > 1e-4 ? toHub.getAngle() : Rotation2d.kZero;
    Translation2d ray =
        distance > 1e-4 ? toHub.div(distance) : new Translation2d(1.0, 0.0);
    if (!isFinite(ray) || ray.getNorm() < 1e-6) {
      ray = new Translation2d(1.0, 0.0);
      bearing = Rotation2d.kZero;
    }

    double maxRpm = positive(in.maxRpm, 5500.0);
    double stationaryRpm = MathUtil.clamp(stationaryRpm(in, clampedDistance), 0.0, maxRpm);
    double metersPerSecondPerRpm =
        MathUtil.clamp(finiteOr(in.metersPerSecondPerRpm, 0.005), 0.001, 0.02);
    double exit = stationaryRpm * metersPerSecondPerRpm;
    double minExit = positive(in.minExitMetersPerSecond, 4.0);
    if (!isFinite(exit) || exit < minExit) {
      exit = minExit;
    }

    // Positive perpendicular speed is to the left of the hub ray. Positive radial speed closes.
    double vLeft = ray.getX() * vy - ray.getY() * vx;
    double vRadial = vx * ray.getX() + vy * ray.getY();

    Translation2d shot = ray.times(exit).minus(new Translation2d(vx, vy));
    if (!isFinite(shot) || shot.getNorm() < 1e-4) {
      shot = ray.times(exit);
    }

    double scale = shot.getNorm() / exit;
    if (!isFinite(scale)) {
      scale = 1.0;
    }
    double minScale = finiteOr(in.minScale, 0.70);
    double maxScale = finiteOr(in.maxScale, 1.40);
    if (maxScale < minScale) {
      maxScale = minScale;
    }
    scale = MathUtil.clamp(scale, minScale, maxScale);

    double rpm = stationaryRpm * scale;
    if (!isFinite(rpm)) {
      rpm = FALLBACK_RPM;
    }
    rpm = MathUtil.clamp(rpm, 0.0, maxRpm);

    double gain = MathUtil.clamp(finiteOrZero(in.leadGainRadiansPerMps), -0.20, 0.20);
    // Positive gain aims against the sideways velocity, same direction as the vector solution.
    double trim = -gain * vLeft;
    Rotation2d aim = shot.getAngle().plus(Rotation2d.fromRadians(trim));
    if (!isFinite(aim.getRadians())) {
      aim = bearing;
    }

    double maxLead = positive(in.maxLeadRadians, Math.toRadians(25.0));
    double delta = Math.IEEEremainder(aim.minus(bearing).getRadians(), Math.PI * 2.0);
    if (!isFinite(delta)) {
      delta = 0.0;
      aim = bearing;
    } else if (Math.abs(delta) > maxLead) {
      delta = Math.copySign(maxLead, delta);
      aim = bearing.plus(Rotation2d.fromRadians(delta));
    }

    if (!isFinite(vLeft)) {
      vLeft = 0.0;
    }
    if (!isFinite(vRadial)) {
      vRadial = 0.0;
    }

    return new Solution(
        future, aim, clampedDistance, rpm, stationaryRpm, delta, vLeft, vRadial, true);
  }

  /** Distance regression, using the caller's function when it returns a finite RPM. */
  private static double stationaryRpm(Input in, double meters) {
    if (in.rpmForDistance != null) {
      try {
        double rpm = in.rpmForDistance.applyAsDouble(meters);
        if (isFinite(rpm)) {
          return rpm;
        }
      } catch (RuntimeException ignored) {
        // Fall through to the built-in curve.
      }
    }
    double base = finiteOr(in.regressionBase, 1350.92838);
    double exp = finiteOr(in.regressionExp, 1.00529);
    double inches = meters * 39.37007874015748;
    double rpm = base * Math.pow(exp, inches);
    if (!isFinite(rpm)) {
      return FALLBACK_RPM;
    }
    return rpm;
  }

  private static double finiteOrZero(double value) {
    return isFinite(value) ? value : 0.0;
  }

  private static double finiteOr(double value, double fallback) {
    return isFinite(value) ? value : fallback;
  }

  private static double positive(double value, double fallback) {
    if (!isFinite(value) || value <= 0.0) {
      return fallback;
    }
    return value;
  }
}
