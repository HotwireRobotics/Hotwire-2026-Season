package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import java.util.function.DoubleUnaryOperator;

/**
 * Aim and flywheel speed for shooting while the chassis is moving.
 *
 * <p>A stopped robot gets the distance regression unchanged. A moving robot solves the fixed-hood
 * trajectory: {@code tan(pitch) * |Δ - v t| = ½ g t² + (hub height - muzzle height)}. The aim is
 * the direction of {@code Δ/t - v}, the horizontal velocity the muzzle must add to the chassis.
 * Flywheel RPM is the regression times that exit speed over the stopped shot's exit speed, so a
 * stopped robot stays on the regression. That ratio is fixed by the equation. There is no hang-time
 * gain, RPM scale, or lead-angle cap. The only ceiling is the flywheel's max RPM. The hub pose
 * itself does not move. The chassis faces the opposite way from {@link Solution#aim}.
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

    /**
     * Ground range the ball flies relative to the moving muzzle, {@code |Δ - v t|}, meters. Equal to
     * {@link #distanceMeters} when the chassis is stopped.
     */
    public final double effectiveDistanceMeters;

    public final double rpm;
    public final double stationaryRpm;

    /** Flight time of the solved lob, seconds. Zero when no real root exists. */
    public final double flightSeconds;

    /** Heading offset from the raw hub bearing, radians. The projectile equation sets this. */
    public final double leadRadians;

    /** Chassis speed to the left of the hub ray, m/s. */
    public final double perpMetersPerSecond;

    /** Chassis speed toward the hub, m/s. Positive means closing. */
    public final double radialMetersPerSecond;

    /** False when the inputs were unusable and this is a safe stand-in. */
    public final boolean live;

    /**
     * @param pose release translation
     * @param aim field direction of the ball, not the chassis heading
     * @param distanceMeters distance to the real hub
     * @param effectiveDistanceMeters ground range {@code |Δ - v t|}
     * @param rpm flywheel setpoint
     * @param stationaryRpm regression RPM at the real hub distance
     * @param flightSeconds flight time of the solved lob
     * @param leadRadians offset from the raw bearing
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

    /**
     * Muzzle in the robot frame, meters. Defaults are {@link Constants.Shooter#kShooterForwardMeters}
     * and {@link Constants.Shooter#kShooterLeftMeters}, the fuel exit copied from {@code
     * Gamepiece.launchFuel(LinearVelocity)}.
     */
    public double shooterForwardMeters = Constants.Shooter.kShooterForwardMeters;

    public double shooterLeftMeters = Constants.Shooter.kShooterLeftMeters;
    public double lookaheadSeconds = Constants.Shooter.kShotLookaheadSeconds;
    public double hoodPitchRadians = Math.toRadians(Constants.Shooter.kHoodPitchDegrees);
    public double launchHeightMeters = Constants.Shooter.kShooterHeightMeters;
    public double hubEntryHeightMeters = Constants.Shooter.kHubEntryHeightMeters;
    public double gravityMetersPerSecondSquared = Constants.Shooter.kGravityMetersPerSecondSquared;

    /** Sensor sanity limit on chassis speed, m/s. Not an aim or RPM gain. */
    public double maxFieldSpeed = 5.5;

    public double maxRpm = 5500.0;
    public double regressionBase = Constants.base;
    public double regressionExp = Constants.exponential;

    /** When set, replaces the built-in distance regression. */
    public DoubleUnaryOperator rpmForDistance;
  }

  /**
   * True when the wrapped heading error is inside {@code toleranceRadians}. This is the check
   * {@code Measure.isNear} does not do: that method subtracts the raw angle magnitudes.
   */
  public static boolean headingsAligned(
      Rotation2d measured, Rotation2d target, double toleranceRadians) {
    if (measured == null || target == null || !isFinite(toleranceRadians)) {
      return false;
    }
    double error = Math.abs(measured.minus(target).getRadians());
    return isFinite(error) && error <= Math.abs(toleranceRadians);
  }

  /**
   * Field heading that points the back of the robot at a fixed hub.
   *
   * <p>The two translations are used as-is. This does not look at velocity, acceleration,
   * lookahead, vision, or a previous solution. Returns null when either translation is unusable or
   * they are the same point, so the caller can hold the current gyro heading instead of commanding
   * a made-up angle.
   *
   * @param robot current odometry translation
   * @param hub fixed hub translation
   * @return chassis heading, or null when there is no bearing
   */
  public static Rotation2d chassisHeading(Translation2d robot, Translation2d hub) {
    if (!isFinite(robot) || !isFinite(hub)) {
      return null;
    }
    double dx = hub.getX() - robot.getX();
    double dy = hub.getY() - robot.getY();
    if (dx * dx + dy * dy <= 1e-6) {
      return null;
    }
    return new Rotation2d(Math.atan2(dy, dx)).plus(Rotation2d.k180deg);
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

    // Release pose is the projected robot center plus the muzzle, in the field frame.
    // The same offset is what fieldVelocity uses for ω×r, so position and speed cannot drift.
    Translation2d offset = shooterOffsetField(in);
    Translation2d release =
        in.robot
            .plus(velocity.times(lookahead))
            .plus(accel.times(0.5 * lookahead * lookahead))
            .plus(offset);
    if (!isFinite(release)) {
      release = in.robot.plus(offset);
    }
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
    // Real geometry. A made-up minimum distance would move both the regression and the lob.
    double distance = Math.max(geometric, 0.0);
    Rotation2d bearing = geometric > 1e-4 ? toHubNow.getAngle() : Rotation2d.kZero;
    Translation2d ray =
        geometric > 1e-4 ? toHubNow.div(geometric) : new Translation2d(1.0, 0.0);
    if (!isFinite(ray) || ray.getNorm() < 1e-6) {
      ray = new Translation2d(1.0, 0.0);
      bearing = Rotation2d.kZero;
    }

    double vLeft = ray.getX() * vRelease.getY() - ray.getY() * vRelease.getX();
    double vRadial = vRelease.getX() * ray.getX() + vRelease.getY() * ray.getY();
    if (!isFinite(vLeft)) {
      vLeft = 0.0;
    }
    if (!isFinite(vRadial)) {
      vRadial = 0.0;
    }

    double maxRpm = positive(in.maxRpm, Constants.Shooter.kMaxRpm);
    double stationary = MathUtil.clamp(stationaryRpm(in, distance), 0.0, maxRpm);
    double pitch = finiteOr(in.hoodPitchRadians, Math.toRadians(Constants.Shooter.kHoodPitchDegrees));
    // Numerical guard so tan and cos stay finite. The hood itself is a fixed 70°.
    pitch = MathUtil.clamp(pitch, 0.05, Math.PI / 2.0 - 0.05);
    double gravity =
        positive(in.gravityMetersPerSecondSquared, Constants.Shooter.kGravityMetersPerSecondSquared);
    double launchZ = finiteOr(in.launchHeightMeters, Constants.Shooter.kShooterHeightMeters);
    double entryZ = finiteOr(in.hubEntryHeightMeters, Constants.Shooter.kHubEntryHeightMeters);
    double dz = entryZ - launchZ;

    Arc arc = null;
    if (geometric > 1e-4 && isFinite(dz)) {
      arc = solveArc(toHubNow, vRelease, pitch, gravity, dz);
    }

    Rotation2d aim = bearing;
    double rpm = stationary;
    double flight = 0.0;
    double effective = distance;
    double lead = 0.0;
    if (arc != null && isFinite(arc.exitMetersPerSecond) && isFinite(arc.aim.getRadians())) {
      // Stopped exit speed from the same equation. The ratio is 1 when v is 0, so the regression
      // is not replaced. It is not a tuned gain: s and s0 are fixed by pitch, gravity, and height.
      Double stoppedExit = stoppedExitSpeed(distance, pitch, gravity, dz);
      boolean stopped = vRelease.getNorm() <= 1e-8;
      if (!stopped && stoppedExit != null && stoppedExit > 1e-6) {
        double scaled = stationary * (arc.exitMetersPerSecond / stoppedExit);
        if (isFinite(scaled)) {
          rpm = scaled;
        }
      } else if (!stopped && isFinite(arc.groundRangeMeters)) {
        // Too close for a stopped lob. The moving root is the same vacuum shot as a stopped
        // robot at the ground range the ball actually flies, so use that regression entry.
        double ranged = stationaryRpm(in, arc.groundRangeMeters);
        if (isFinite(ranged)) {
          rpm = ranged;
        }
      }
      if (!stopped) {
        aim = arc.aim;
        lead = Math.IEEEremainder(aim.minus(bearing).getRadians(), Math.PI * 2.0);
        if (!isFinite(lead)) {
          lead = 0.0;
          aim = bearing;
        }
      }
      flight = arc.timeSeconds;
      if (isFinite(arc.groundRangeMeters)) {
        effective = arc.groundRangeMeters;
      }
    }
    if (!isFinite(rpm)) {
      rpm = FALLBACK_RPM;
    }
    rpm = MathUtil.clamp(rpm, 0.0, maxRpm);

    return new Solution(
        release, aim, distance, effective, rpm, stationary, flight, lead, vLeft, vRadial, true);
  }

  /**
   * Muzzle translation in the field frame. Robot frame is +X forward and +Y left, rotated by the
   * gyro heading. A non-finite offset becomes zero so the shot falls back to the robot center.
   */
  private static Translation2d shooterOffsetField(Input in) {
    Translation2d offset =
        new Translation2d(
                finiteOrZero(in.shooterForwardMeters), finiteOrZero(in.shooterLeftMeters))
            .rotateBy(Rotation2d.fromRadians(finiteOrZero(in.headingRadians)));
    if (!isFinite(offset)) {
      return Translation2d.kZero;
    }
    return offset;
  }

  /**
   * Field velocity of the muzzle. Chassis spin adds ω×r using {@link #shooterOffsetField}, the
   * same vector added to the release translation. Non-finite components are dropped.
   */
  private static Translation2d fieldVelocity(Input in, double maxSpeed) {
    double vx = finiteOrZero(in.vxMetersPerSecond);
    double vy = finiteOrZero(in.vyMetersPerSecond);
    double omega = MathUtil.clamp(finiteOrZero(in.omegaRadiansPerSecond), -12.0, 12.0);
    Translation2d offset = shooterOffsetField(in);
    if (offset.getNorm() > 1e-4 && omega != 0.0) {
      // ω × r for ω about +Z: (-ω y, ω x).
      vx += -omega * offset.getY();
      vy += omega * offset.getX();
    }
    return clampVector(new Translation2d(vx, vy), maxSpeed);
  }

  /**
   * One root of {@code tan(pitch) * |Δ - v t| = ½ g t² + Δz}. {@code aim} is the direction of
   * {@code Δ/t - v}. {@code exitMetersPerSecond} is that vector's magnitude divided by {@code
   * cos(pitch)}.
   */
  private static final class Arc {
    final double timeSeconds;
    final Rotation2d aim;
    final double exitMetersPerSecond;
    final double groundRangeMeters;

    Arc(
        double timeSeconds,
        Rotation2d aim,
        double exitMetersPerSecond,
        double groundRangeMeters) {
      this.timeSeconds = timeSeconds;
      this.aim = aim;
      this.exitMetersPerSecond = exitMetersPerSecond;
      this.groundRangeMeters = groundRangeMeters;
    }
  }

  /**
   * Descending fixed-hood solution. A stopped chassis uses the closed form. A moving chassis
   * brackets {@link #residual} and keeps the root nearest the stopped flight time, which is the
   * lob that continues from the stationary shot. Returns null when a 70° ball cannot come down
   * through the hub mouth.
   */
  private static Arc solveArc(
      Translation2d delta, Translation2d velocity, double pitch, double gravity, double dz) {
    double cos = Math.cos(pitch);
    double tan = Math.tan(pitch);
    if (!(cos > 1e-4) || !isFinite(tan) || !isFinite(delta) || !isFinite(velocity) || !(gravity > 0.0)) {
      return null;
    }
    double distance = delta.getNorm();
    if (velocity.getNorm() <= 1e-8) {
      Double flight = stoppedFlightSeconds(distance, pitch, gravity, dz);
      if (flight == null) {
        return null;
      }
      double exit = (distance / flight) / cos;
      if (!isFinite(exit)) {
        return null;
      }
      return new Arc(flight, delta.getAngle(), exit, distance);
    }

    double preferred = Double.NaN;
    Double stopped = stoppedFlightSeconds(distance, pitch, gravity, dz);
    if (stopped != null) {
      preferred = stopped;
    }
    // Earliest a ball launched from below the mouth can be descending through it.
    double tMin = dz > 0.0 ? Math.sqrt(2.0 * dz / gravity) : 1e-3;
    if (!isFinite(tMin) || tMin < 1e-3) {
      tMin = 1e-3;
    }
    // Past any 70° lob the chassis can still reach. After this, -½gt² keeps the residual negative.
    double tMax = 8.0;
    double step = 0.02;
    Arc best = null;
    double bestScore = Double.POSITIVE_INFINITY;
    double tPrev = tMin;
    double fPrev = residual(tPrev, delta, velocity, tan, gravity, dz);
    for (double t = tMin + step; t <= tMax + 1e-9; t += step) {
      double f = residual(t, delta, velocity, tan, gravity, dz);
      Double root = crossing(tPrev, t, fPrev, f, delta, velocity, tan, gravity, dz);
      if (root != null) {
        Arc arc = arcAt(root, delta, velocity, cos);
        if (arc != null) {
          double score = isFinite(preferred) ? Math.abs(arc.timeSeconds - preferred) : -arc.timeSeconds;
          if (best == null || score < bestScore) {
            best = arc;
            bestScore = score;
          }
        }
      }
      tPrev = t;
      fPrev = f;
    }
    if (isFinite(fPrev) && fPrev == 0.0) {
      Arc arc = arcAt(tPrev, delta, velocity, cos);
      if (arc != null) {
        double score = isFinite(preferred) ? Math.abs(arc.timeSeconds - preferred) : -arc.timeSeconds;
        if (best == null || score < bestScore) {
          best = arc;
        }
      }
    }
    return best;
  }

  /**
   * Flight time for a stopped shot that descends through {@code dz}. Null when the mouth is above
   * the 70° launch ray or the ball would still be climbing at the hub.
   */
  private static Double stoppedFlightSeconds(
      double distance, double pitch, double gravity, double dz) {
    double tan = Math.tan(pitch);
    if (!(distance > 1e-6) || !(tan > 0.0) || !(gravity > 0.0) || !isFinite(dz)) {
      return null;
    }
    double reach = distance * tan - dz;
    // Peak height is above the mouth only when reach > dz, i.e. distance * tan(pitch) > 2 dz.
    if (!(reach > dz) || !isFinite(reach)) {
      return null;
    }
    double t2 = 2.0 * reach / gravity;
    if (!(t2 > 0.0) || !isFinite(t2)) {
      return null;
    }
    return Math.sqrt(t2);
  }

  /**
   * Exit speed along the hood for a stopped shot at {@code distance}. Same equation as {@link
   * #solveArc} with v = 0.
   */
  private static Double stoppedExitSpeed(double distance, double pitch, double gravity, double dz) {
    Double flight = stoppedFlightSeconds(distance, pitch, gravity, dz);
    if (flight == null) {
      return null;
    }
    double cos = Math.cos(pitch);
    if (!(cos > 1e-4)) {
      return null;
    }
    double exit = (distance / flight) / cos;
    if (!isFinite(exit) || exit <= 0.0) {
      return null;
    }
    return exit;
  }

  /** {@code tan(pitch) * |Δ - v t| - ½ g t² - Δz}. Zero at a hub-mouth crossing. */
  private static double residual(
      double t,
      Translation2d delta,
      Translation2d velocity,
      double tan,
      double gravity,
      double dz) {
    double range = delta.minus(velocity.times(t)).getNorm();
    return tan * range - 0.5 * gravity * t * t - dz;
  }

  /** Time in {@code (t0, t1)} where {@link #residual} changes sign, or null. */
  private static Double crossing(
      double t0,
      double t1,
      double f0,
      double f1,
      Translation2d delta,
      Translation2d velocity,
      double tan,
      double gravity,
      double dz) {
    if (isFinite(f0) && f0 == 0.0) {
      return t0;
    }
    if (!isFinite(f0) || !isFinite(f1) || f0 * f1 >= 0.0) {
      return null;
    }
    double lo = t0;
    double hi = t1;
    double flo = f0;
    for (int i = 0; i < 50; i++) {
      double mid = 0.5 * (lo + hi);
      double fm = residual(mid, delta, velocity, tan, gravity, dz);
      if (!isFinite(fm)) {
        return null;
      }
      if (flo * fm <= 0.0) {
        hi = mid;
      } else {
        lo = mid;
        flo = fm;
      }
    }
    double root = 0.5 * (lo + hi);
    return isFinite(root) ? root : null;
  }

  /** Muzzle velocity and aim implied by a flight time that already satisfies the height equation. */
  private static Arc arcAt(double t, Translation2d delta, Translation2d velocity, double cos) {
    if (!(t > 1e-4) || !isFinite(t) || !(cos > 1e-4)) {
      return null;
    }
    Translation2d added = delta.div(t).minus(velocity);
    if (!isFinite(added)) {
      return null;
    }
    double horizontal = added.getNorm();
    if (!(horizontal > 1e-6)) {
      return null;
    }
    double exit = horizontal / cos;
    double ground = horizontal * t;
    if (!isFinite(exit) || !isFinite(ground)) {
      return null;
    }
    return new Arc(t, added.getAngle(), exit, ground);
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
    double base = finiteOr(in.regressionBase, Constants.base);
    double exp = finiteOr(in.regressionExp, Constants.exponential);
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
