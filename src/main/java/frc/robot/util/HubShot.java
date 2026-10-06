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
 * <p>A stopped robot gets the distance regression unchanged. A moving robot uses the fixed-hood
 * virtual-target method: project the release point forward by the shot latency, estimate flight
 * time from hood pitch and exit speed, then move the hub opposite the chassis velocity for that
 * flight and iterate until the two agree. RPM is the regression at the distance to that virtual
 * hub, so forward and backward speed follow the real distance curve. Heading aims at the virtual
 * hub. Every returned number is finite, and the heading offset and RPM scale are clamped.
 */
public final class HubShot {

  /** Ferry RPM used when the inputs cannot produce a shot. */
  public static final double FALLBACK_RPM = 2400.0;

  /** Largest chassis acceleration the release prediction will trust, m/s^2. */
  private static final double MAX_ACCEL = 8.0;

  private HubShot() {}

  /** One solved shot. Every numeric field is finite. */
  public static final class Solution {
    /** Release translation the shot was solved from. */
    public final Translation2d pose;

    public final Rotation2d aim;

    /** Distance from the release point to the real hub, meters. */
    public final double distanceMeters;

    /** Distance from the release point to the virtual hub, meters. This selects RPM. */
    public final double effectiveDistanceMeters;

    public final double rpm;
    public final double stationaryRpm;

    /** Flight time used for the virtual hub, seconds. */
    public final double flightSeconds;

    /** Heading offset from the raw hub bearing, radians, after clamping. */
    public final double leadRadians;

    /** Chassis speed to the left of the hub ray, m/s. */
    public final double perpMetersPerSecond;

    /** Chassis speed toward the hub, m/s. Positive means closing. */
    public final double radialMetersPerSecond;

    /** False when the inputs were unusable and this is a safe stand-in. */
    public final boolean live;

    /**
     * @param pose release translation
     * @param aim field heading to hold
     * @param distanceMeters distance to the real hub
     * @param effectiveDistanceMeters distance used for the regression
     * @param rpm flywheel setpoint
     * @param stationaryRpm regression RPM at the real hub distance
     * @param flightSeconds flight time of the converged shot
     * @param leadRadians clamped offset from the raw bearing
     * @param perpMetersPerSecond sideways speed, positive to the left of the ray
     * @param radialMetersPerSecond speed toward the hub
     * @param live true when this was solved from finite inputs
     */
    public Solution(
        Translation2d pose,
        Rotation2d aim,
        double distanceMeters,
        double effectiveDistanceMeters,
        double rpm,
        double stationaryRpm,
        double flightSeconds,
        double leadRadians,
        double perpMetersPerSecond,
        double radialMetersPerSecond,
        boolean live) {
      this.pose = pose;
      this.aim = aim;
      this.distanceMeters = distanceMeters;
      this.effectiveDistanceMeters = effectiveDistanceMeters;
      this.rpm = rpm;
      this.stationaryRpm = stationaryRpm;
      this.flightSeconds = flightSeconds;
      this.leadRadians = leadRadians;
      this.perpMetersPerSecond = perpMetersPerSecond;
      this.radialMetersPerSecond = radialMetersPerSecond;
      this.live = live;
    }
  }

  /**
   * Low-pass on field velocity. Same-timestamp updates return the previous sample so several shot
   * solves in one robot loop do not wipe the filter. A non-finite sample is ignored.
   */
  public static final class VelocityFilter {
    private Translation2d filtered = Translation2d.kZero;
    private Translation2d acceleration = Translation2d.kZero;
    private double lastTimestamp = Double.NaN;

    /** Filtered field velocity, m/s. */
    public Translation2d velocity() {
      return filtered;
    }

    /** Derivative of the filtered velocity, m/s^2, clamped. */
    public Translation2d acceleration() {
      return acceleration;
    }

    /**
     * @param vx field x, m/s
     * @param vy field y, m/s
     * @param timestampSeconds FPGA timestamp
     * @param tauSeconds low-pass time constant; 0 disables the filter
     */
    public Translation2d update(double vx, double vy, double timestampSeconds, double tauSeconds) {
      if (!Double.isFinite(vx) || !Double.isFinite(vy) || !Double.isFinite(timestampSeconds)) {
        return filtered;
      }
      Translation2d sample = new Translation2d(vx, vy);
      if (!Double.isFinite(lastTimestamp)) {
        seed(sample, timestampSeconds);
        return filtered;
      }
      double dt = timestampSeconds - lastTimestamp;
      // Another solve in the same cycle. Keep the value this cycle already accepted.
      if (dt >= -1e-4 && dt < 1e-4) {
        return filtered;
      }
      // Clock jump or a long gap. Trust the new sample and drop stale acceleration.
      if (dt < 0.0 || dt > 0.25) {
        seed(sample, timestampSeconds);
        return filtered;
      }
      double alpha = 1.0;
      if (Double.isFinite(tauSeconds) && tauSeconds > 1e-4) {
        alpha = 1.0 - Math.exp(-dt / tauSeconds);
      }
      alpha = MathUtil.clamp(alpha, 0.0, 1.0);
      Translation2d next = filtered.plus(sample.minus(filtered).times(alpha));
      if (!isFinite(next)) {
        seed(sample, timestampSeconds);
        return filtered;
      }
      Translation2d accel = next.minus(filtered).div(dt);
      double accelNorm = accel.getNorm();
      if (!Double.isFinite(accelNorm)) {
        accel = Translation2d.kZero;
      } else if (accelNorm > MAX_ACCEL) {
        accel = accel.times(MAX_ACCEL / accelNorm);
      }
      acceleration = accel;
      filtered = next;
      lastTimestamp = timestampSeconds;
      return filtered;
    }

    private void seed(Translation2d sample, double timestampSeconds) {
      filtered = sample;
      acceleration = Translation2d.kZero;
      lastTimestamp = timestampSeconds;
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
    public double axMetersPerSecondSquared;
    public double ayMetersPerSecondSquared;
    public double omegaRadiansPerSecond;
    public double headingRadians;
    public double shooterForwardMeters;
    public double shooterLeftMeters;
    public double lookaheadSeconds = 0.10;
    public double metersPerSecondPerRpm = 0.005;
    public double hoodPitchRadians = Math.toRadians(55.0);
    public double leadGainRadiansPerMps = 0.0;
    public double minScale = 0.70;
    public double maxScale = 1.40;
    public double maxLeadRadians = Math.toRadians(25.0);
    public double maxFieldSpeed = 5.5;
    public double minExitMetersPerSecond = 4.0;
    public double minDistanceMeters = 0.30;
    public double maxDistanceMeters = 8.0;
    public double maxRpm = 5500.0;
    public double minFlightSeconds = 0.12;
    public double maxFlightSeconds = 1.10;
    public int iterations = 6;
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
        0.30,
        FALLBACK_RPM,
        FALLBACK_RPM,
        0.0,
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
    Translation2d velocity = fieldVelocity(in, maxSpeed);
    Translation2d accel =
        clampVector(
            new Translation2d(
                finiteOrZero(in.axMetersPerSecondSquared),
                finiteOrZero(in.ayMetersPerSecondSquared)),
            MAX_ACCEL);
    double lookahead = MathUtil.clamp(finiteOrZero(in.lookaheadSeconds), 0.0, 0.40);

    // Where the ball leaves, and how fast the chassis is moving then.
    Translation2d release =
        in.robot
            .plus(velocity.times(lookahead))
            .plus(accel.times(0.5 * lookahead * lookahead));
    if (!isFinite(release)) {
      release = in.robot;
    }
    Translation2d vRelease = clampVector(velocity.plus(accel.times(lookahead)), maxSpeed);

    Translation2d toHubNow = in.hub.minus(release);
    if (!isFinite(toHubNow)) {
      return fallback();
    }
    double geometric = toHubNow.getNorm();
    if (!isFinite(geometric)) {
      return fallback();
    }
    double minDistance = positive(in.minDistanceMeters, 0.30);
    double maxDistance = Math.max(minDistance, positive(in.maxDistanceMeters, 8.0));
    double distance = MathUtil.clamp(Math.max(geometric, 0.0), minDistance, maxDistance);
    Rotation2d bearing = geometric > 1e-4 ? toHubNow.getAngle() : Rotation2d.kZero;
    Translation2d ray =
        geometric > 1e-4 ? toHubNow.div(geometric) : new Translation2d(1.0, 0.0);
    if (!isFinite(ray) || ray.getNorm() < 1e-6) {
      ray = new Translation2d(1.0, 0.0);
      bearing = Rotation2d.kZero;
    }

    int steps = (int) MathUtil.clamp(in.iterations, 1, 8);
    Translation2d virtual = in.hub;
    double flight = positive(in.minFlightSeconds, 0.12);
    double shiftCap = maxSpeed * positive(in.maxFlightSeconds, 1.10);
    for (int i = 0; i < steps; i++) {
      double virtualDistance = virtual.minus(release).getNorm();
      if (!isFinite(virtualDistance)) {
        virtual = in.hub;
        break;
      }
      virtualDistance = MathUtil.clamp(Math.max(virtualDistance, 0.0), minDistance, maxDistance);
      flight = flightSeconds(in, virtualDistance);
      Translation2d shift = clampVector(vRelease.times(flight), shiftCap);
      Translation2d next = in.hub.minus(shift);
      if (!isFinite(next)) {
        break;
      }
      // Half-step a large jump so a noisy speed cannot oscillate the virtual hub.
      if (i > 0) {
        Translation2d step = next.minus(virtual);
        double stepNorm = step.getNorm();
        if (Double.isFinite(stepNorm) && stepNorm > 0.75) {
          next = virtual.plus(step.times(0.5));
        }
      }
      virtual = next;
    }

    Translation2d toVirtual = virtual.minus(release);
    if (!isFinite(toVirtual)) {
      toVirtual = toHubNow;
      virtual = in.hub;
    }
    double effective = toVirtual.getNorm();
    if (!isFinite(effective)) {
      effective = distance;
    }
    effective = MathUtil.clamp(Math.max(effective, 0.0), minDistance, maxDistance);
    flight = flightSeconds(in, effective);

    double maxRpm = positive(in.maxRpm, 5500.0);
    double stationaryRpm = MathUtil.clamp(stationaryRpm(in, distance), 0.0, maxRpm);
    double movingRpm = MathUtil.clamp(stationaryRpm(in, effective), 0.0, maxRpm);
    double scale = stationaryRpm > 1.0 ? movingRpm / stationaryRpm : 1.0;
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

    double vLeft = ray.getX() * vRelease.getY() - ray.getY() * vRelease.getX();
    double vRadial = vRelease.getX() * ray.getX() + vRelease.getY() * ray.getY();
    if (!isFinite(vLeft)) {
      vLeft = 0.0;
    }
    if (!isFinite(vRadial)) {
      vRadial = 0.0;
    }

    Rotation2d aim = toVirtual.getNorm() > 1e-4 ? toVirtual.getAngle() : bearing;
    double gain = MathUtil.clamp(finiteOrZero(in.leadGainRadiansPerMps), -0.20, 0.20);
    aim = aim.plus(Rotation2d.fromRadians(-gain * vLeft));
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

    return new Solution(
        release,
        aim,
        distance,
        effective,
        rpm,
        stationaryRpm,
        flight,
        delta,
        vLeft,
        vRadial,
        true);
  }

  /**
   * Field velocity of the muzzle. Chassis spin adds a tangential speed when the shooter is not at
   * the robot center. Non-finite components are dropped.
   */
  private static Translation2d fieldVelocity(Input in, double maxSpeed) {
    double vx = finiteOrZero(in.vxMetersPerSecond);
    double vy = finiteOrZero(in.vyMetersPerSecond);
    double omega = MathUtil.clamp(finiteOrZero(in.omegaRadiansPerSecond), -12.0, 12.0);
    double heading = finiteOrZero(in.headingRadians);
    Translation2d offset =
        new Translation2d(
                finiteOrZero(in.shooterForwardMeters), finiteOrZero(in.shooterLeftMeters))
            .rotateBy(Rotation2d.fromRadians(heading));
    if (isFinite(offset) && offset.getNorm() > 1e-4 && omega != 0.0) {
      vx += -omega * offset.getY();
      vy += omega * offset.getX();
    }
    return clampVector(new Translation2d(vx, vy), maxSpeed);
  }

  /**
   * Time for a fixed-hood shot to cover {@code meters}. Horizontal speed is the exit speed times
   * cos(pitch). The result is clamped so a flat or vertical hood cannot explode the virtual hub.
   */
  private static double flightSeconds(Input in, double meters) {
    double rpm = Math.max(stationaryRpm(in, meters), 0.0);
    double perRpm = MathUtil.clamp(finiteOr(in.metersPerSecondPerRpm, 0.005), 0.001, 0.02);
    double exit = rpm * perRpm;
    double minExit = positive(in.minExitMetersPerSecond, 4.0);
    if (!isFinite(exit) || exit < minExit) {
      exit = minExit;
    }
    double pitch = finiteOr(in.hoodPitchRadians, Math.toRadians(55.0));
    pitch = MathUtil.clamp(pitch, Math.toRadians(20.0), Math.toRadians(75.0));
    double horizontal = exit * Math.cos(pitch);
    if (!isFinite(horizontal) || horizontal < 2.0) {
      horizontal = 2.0;
    }
    double flight = meters / horizontal;
    if (!isFinite(flight)) {
      flight = positive(in.minFlightSeconds, 0.12);
    }
    return MathUtil.clamp(
        flight, positive(in.minFlightSeconds, 0.12), positive(in.maxFlightSeconds, 1.10));
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

  /** Shrink a vector so its norm is at most {@code max}. A non-finite vector becomes zero. */
  private static Translation2d clampVector(Translation2d vector, double max) {
    if (!isFinite(vector) || !isFinite(max) || max <= 0.0) {
      return Translation2d.kZero;
    }
    double norm = vector.getNorm();
    if (!isFinite(norm)) {
      return Translation2d.kZero;
    }
    if (norm <= max) {
      return vector;
    }
    return vector.times(max / norm);
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
