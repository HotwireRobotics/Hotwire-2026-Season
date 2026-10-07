package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import org.junit.jupiter.api.Test;

/** Pure checks for moving-shot aim and RPM. These do not start the robot. */
public class HubShotTest {

  private static HubShot.Input still() {
    HubShot.Input in = new HubShot.Input();
    in.robot = new Translation2d(0.0, 0.0);
    in.hub = new Translation2d(4.0, 0.0);
    in.lookaheadSeconds = 0.0;
    return in;
  }

  private static double curve(double meters) {
    return Constants.base * Math.pow(Constants.exponential, Units.metersToInches(meters));
  }

  @Test
  public void stationaryMatchesRegression() {
    HubShot.Solution shot = HubShot.solve(still());
    assertTrue(shot.live);
    assertEquals(0.0, shot.aim.getRadians(), 1e-9);
    assertEquals(0.0, shot.leadRadians, 1e-9);
    // Heading 0, rear muzzle: distance is hub range minus the (negative) forward offset.
    double releaseDistance = 4.0 - Constants.Shooter.kShooterForwardMeters;
    assertEquals(releaseDistance, shot.distanceMeters, 1e-9);
    assertEquals(curve(releaseDistance), shot.stationaryRpm, 1e-6);
    assertEquals(shot.stationaryRpm, shot.rpm, 1e-6);
    double pitch = Math.toRadians(Constants.Shooter.kHoodPitchDegrees);
    double dz = Constants.Shooter.kHubEntryHeightMeters - Constants.Shooter.kShooterHeightMeters;
    double reach = releaseDistance * Math.tan(pitch) - dz;
    double flight =
        Math.sqrt(2.0 * reach / Constants.Shooter.kGravityMetersPerSecondSquared);
    assertEquals(flight, shot.flightSeconds, 1e-9);
    assertEquals(releaseDistance, shot.effectiveDistanceMeters, 1e-9);
  }

  @Test
  public void drivingTowardHubLowersRpm() {
    HubShot.Input in = still();
    in.vxMetersPerSecond = 2.0;
    HubShot.Solution moving = HubShot.solve(in);
    HubShot.Solution parked = HubShot.solve(still());
    assertTrue(moving.rpm < parked.rpm);
    assertEquals(0.0, moving.aim.getRadians(), 1e-9);
    assertTrue(moving.radialMetersPerSecond > 0.0);
  }

  @Test
  public void drivingAwayRaisesRpm() {
    HubShot.Input in = still();
    in.vxMetersPerSecond = -2.0;
    HubShot.Solution moving = HubShot.solve(in);
    assertTrue(moving.rpm > HubShot.solve(still()).rpm);
    assertTrue(moving.radialMetersPerSecond < 0.0);
  }

  @Test
  public void sidewaysAimsBehindMotion() {
    HubShot.Input in = still();
    in.vyMetersPerSecond = 2.0;
    HubShot.Solution shot = HubShot.solve(in);
    HubShot.Solution parked = HubShot.solve(still());
    // Moving to the left of the ray, so the chassis aims to the right of the hub.
    // 2 m/s at 4 m hangs long enough that the lead is well past the old 25° cap.
    assertTrue(shot.aim.getRadians() < Math.toRadians(-30.0));
    assertTrue(shot.perpMetersPerSecond > 0.0);
    assertTrue(shot.rpm > parked.rpm);
  }

  @Test
  public void fastRetreatClearsTheOldRpmCap() {
    HubShot.Input in = still();
    in.vxMetersPerSecond = -3.0;
    HubShot.Solution shot = HubShot.solve(in);
    // The equation, not a scale clamp, raises RPM. The motor ceiling is the only limit.
    assertTrue(shot.rpm > shot.stationaryRpm * 1.40);
    assertTrue(shot.rpm < Constants.Shooter.kMaxRpm);
    assertEquals(0.0, shot.aim.getRadians(), 1e-6);
  }

  @Test
  public void fastCloseDropsBelowTheOldRpmFloor() {
    HubShot.Input in = still();
    in.vxMetersPerSecond = 3.0;
    HubShot.Solution shot = HubShot.solve(in);
    assertTrue(shot.rpm < shot.stationaryRpm * 0.70);
    assertTrue(shot.rpm > 0.0);
    assertEquals(0.0, shot.aim.getRadians(), 1e-6);
  }

  @Test
  public void lookaheadShortensAClosingShot() {
    HubShot.Input parked = still();
    HubShot.Input moving = still();
    moving.vxMetersPerSecond = 2.0;
    moving.lookaheadSeconds = 0.20;
    assertTrue(HubShot.solve(moving).distanceMeters < HubShot.solve(parked).distanceMeters);
  }

  @Test
  public void nonFiniteVelocityMatchesStopped() {
    HubShot.Input in = still();
    in.vxMetersPerSecond = Double.NaN;
    in.vyMetersPerSecond = Double.POSITIVE_INFINITY;
    HubShot.Solution shot = HubShot.solve(in);
    HubShot.Solution parked = HubShot.solve(still());
    assertTrue(shot.live);
    assertEquals(parked.rpm, shot.rpm, 1e-6);
    assertEquals(parked.aim.getRadians(), shot.aim.getRadians(), 1e-9);
  }

  @Test
  public void impossibleSpeedHoldsTheHubBearing() {
    HubShot.Input in = still();
    in.vyMetersPerSecond = 50.0;
    in.maxFieldSpeed = 50.0;
    HubShot.Solution shot = HubShot.solve(in);
    HubShot.Solution parked = HubShot.solve(still());
    assertTrue(shot.live);
    assertEquals(0.0, shot.aim.getRadians(), 1e-9);
    assertEquals(0.0, shot.leadRadians, 1e-9);
    assertEquals(parked.rpm, shot.rpm, 1e-6);
    assertTrue(shot.rpm <= Constants.Shooter.kMaxRpm);
  }

  @Test
  public void hardSidewaysLeadIsNotCapped() {
    HubShot.Input in = still();
    in.vyMetersPerSecond = 5.5;
    HubShot.Solution shot = HubShot.solve(in);
    // Old compensation stopped the lead at 70°. The equation does not.
    assertTrue(shot.aim.getRadians() < Math.toRadians(-70.0));
    assertEquals(Constants.Shooter.kMaxRpm, shot.rpm, 1e-6);
  }

  @Test
  public void closeRetreatUsesTheRangeItActuallyFlies() {
    HubShot.Input in = still();
    // Muzzle is 0.183 m behind center, so this hub is about 0.50 m from the release point.
    // A stopped 70° lob cannot descend into the mouth from there.
    in.hub = new Translation2d(0.317, 0.0);
    HubShot.Solution parked = HubShot.solve(in);
    assertEquals(0.0, parked.flightSeconds, 1e-9);
    in.vxMetersPerSecond = -2.0;
    HubShot.Solution shot = HubShot.solve(in);
    assertTrue(shot.flightSeconds > 0.5);
    assertTrue(shot.effectiveDistanceMeters > shot.distanceMeters);
    assertTrue(shot.rpm > parked.rpm);
  }

  @Test
  public void movingShotLandsOnTheHubInVacuum() {
    HubShot.Input in = still();
    in.vyMetersPerSecond = 2.0;
    HubShot.Solution shot = HubShot.solve(in);
    double pitch = Math.toRadians(Constants.Shooter.kHoodPitchDegrees);
    double gravity = Constants.Shooter.kGravityMetersPerSecondSquared;
    double dz = Constants.Shooter.kHubEntryHeightMeters - Constants.Shooter.kShooterHeightMeters;
    double reach = shot.distanceMeters * Math.tan(pitch) - dz;
    double stoppedFlight = Math.sqrt(2.0 * reach / gravity);
    double stoppedExit = (shot.distanceMeters / stoppedFlight) / Math.cos(pitch);
    double exit = stoppedExit * (shot.rpm / shot.stationaryRpm);
    double horizontal = exit * Math.cos(pitch);
    double t = shot.flightSeconds;
    double x = shot.pose.getX() + (in.vxMetersPerSecond + horizontal * Math.cos(shot.aim.getRadians())) * t;
    double y = shot.pose.getY() + (in.vyMetersPerSecond + horizontal * Math.sin(shot.aim.getRadians())) * t;
    assertEquals(in.hub.getX(), x, 1e-4);
    assertEquals(in.hub.getY(), y, 1e-4);
    double z =
        Constants.Shooter.kShooterHeightMeters + exit * Math.sin(pitch) * t - 0.5 * gravity * t * t;
    assertEquals(Constants.Shooter.kHubEntryHeightMeters, z, 1e-4);
    assertTrue(exit * Math.sin(pitch) - gravity * t < 0.0);
  }

  @Test
  public void badPoseDoesNotThrow() {
    HubShot.Input in = still();
    in.robot = new Translation2d(Double.NaN, 0.0);
    HubShot.Solution shot = HubShot.solve(in);
    assertNotNull(shot);
    assertFalse(shot.live);
    assertTrue(Double.isFinite(shot.rpm));
    assertTrue(Double.isFinite(shot.aim.getRadians()));

    shot = HubShot.solve(null);
    assertFalse(shot.live);
    assertEquals(HubShot.FALLBACK_RPM, shot.rpm, 1e-9);
  }

  @Test
  public void onTopOfHubStaysFinite() {
    HubShot.Input in = still();
    in.robot = new Translation2d(4.0, 0.0);
    HubShot.Solution shot = HubShot.solve(in);
    assertTrue(shot.live);
    // Robot center is on the hub. The muzzle is still the rear offset away, which is inside the
    // minimum range of a 70° descending lob, so the shot holds the hub bearing and the regression.
    assertEquals(Math.abs(Constants.Shooter.kShooterForwardMeters), shot.distanceMeters, 1e-9);
    assertEquals(0.0, shot.aim.getRadians(), 1e-9);
    assertEquals(0.0, shot.flightSeconds, 1e-9);
    assertTrue(Double.isFinite(shot.rpm));
    assertTrue(shot.rpm > 0.0);
  }

  @Test
  public void closingSpeedShortensTheVirtualHub() {
    HubShot.Input in = still();
    in.vxMetersPerSecond = 2.0;
    HubShot.Solution shot = HubShot.solve(in);
    assertTrue(shot.effectiveDistanceMeters < shot.distanceMeters - 0.2);
    assertTrue(shot.flightSeconds > 0.12);
    assertEquals(curve(shot.distanceMeters), shot.stationaryRpm, 1e-6);
  }

  @Test
  public void fartherShotsStayInTheAirLonger() {
    HubShot.Input near = still();
    near.hub = new Translation2d(2.0, 0.0);
    HubShot.Input far = still();
    far.hub = new Translation2d(6.0, 0.0);
    assertTrue(HubShot.solve(far).flightSeconds > HubShot.solve(near).flightSeconds);
  }

  @Test
  public void releasePoseUsesTheFuelExitOffset() {
    // Gamepiece.launchFuel(LinearVelocity): Meters.of(-0.183302), zero fixed left, Inches.of(14.759196).
    assertEquals(-0.183302, Constants.Shooter.kShooterForwardMeters, 0.0);
    assertEquals(0.0, Constants.Shooter.kShooterLeftMeters, 0.0);
    assertEquals(Units.inchesToMeters(14.759196), Constants.Shooter.kShooterHeightMeters, 0.0);

    HubShot.Input in = still();
    in.headingRadians = 0.0;
    HubShot.Solution shot = HubShot.solve(in);
    assertEquals(-0.183302, shot.pose.getX(), 1e-9);
    assertEquals(0.0, shot.pose.getY(), 1e-9);

    // 90° CCW takes robot-frame (-0.183302, 0) to field (0, -0.183302).
    in.headingRadians = Math.PI / 2.0;
    shot = HubShot.solve(in);
    assertEquals(0.0, shot.pose.getX(), 1e-9);
    assertEquals(-0.183302, shot.pose.getY(), 1e-9);

    // A side component rotates with the same heading. +Y left, 90° CCW: (x, y) -> (-y, x).
    in.shooterLeftMeters = 0.10;
    shot = HubShot.solve(in);
    assertEquals(-0.10, shot.pose.getX(), 1e-9);
    assertEquals(-0.183302, shot.pose.getY(), 1e-9);

    // ω×r uses that same field offset. Heading 0, ω = 2: vy = ω * forward.
    in.headingRadians = 0.0;
    in.shooterLeftMeters = 0.0;
    in.omegaRadiansPerSecond = 2.0;
    shot = HubShot.solve(in);
    assertEquals(2.0 * -0.183302, shot.perpMetersPerSecond, 1e-9);
  }

  @Test
  public void yawAtAForwardShooterAimsBehindTheSwing() {
    HubShot.Input in = still();
    in.omegaRadiansPerSecond = 2.0;
    in.shooterForwardMeters = 0.30;
    in.headingRadians = 0.0;
    HubShot.Solution shot = HubShot.solve(in);
    assertTrue(shot.aim.getRadians() < 0.0);
    assertTrue(shot.perpMetersPerSecond > 0.0);
  }

  @Test
  public void accelerationAtReleaseChangesTheShot() {
    HubShot.Input coasting = still();
    coasting.lookaheadSeconds = 0.20;
    HubShot.Input speeding = still();
    speeding.lookaheadSeconds = 0.20;
    speeding.axMetersPerSecondSquared = 5.0;
    assertTrue(HubShot.solve(speeding).rpm < HubShot.solve(coasting).rpm);
  }

  @Test
  public void headingCheckWrapsAroundTheCircle() {
    Rotation2d measured = Rotation2d.fromDegrees(179.0);
    Rotation2d target = Rotation2d.fromDegrees(-179.0);
    assertTrue(HubShot.headingsAligned(measured, target, Math.toRadians(4.0)));
    assertFalse(HubShot.headingsAligned(Rotation2d.fromDegrees(0.0), Rotation2d.fromDegrees(10.0), Math.toRadians(4.0)));
  }

  @Test
  public void chassisHeadingPointsTheBackAtTheStaticHub() {
    // Hub due east of the robot. The ball travels at 0°, so the chassis faces 180°.
    Rotation2d east =
        HubShot.chassisHeading(new Translation2d(0.0, 0.0), new Translation2d(4.0, 0.0));
    assertNotNull(east);
    assertEquals(Math.PI, east.getRadians(), 1e-9);

    // Hub due north. Chassis faces south.
    Rotation2d north =
        HubShot.chassisHeading(new Translation2d(1.0, 1.0), new Translation2d(1.0, 5.0));
    assertNotNull(north);
    assertEquals(-Math.PI / 2.0, north.getRadians(), 1e-9);

    // A moving shot's virtual hub is a different direction. The chassis heading does not follow it.
    HubShot.Input moving = still();
    moving.vyMetersPerSecond = 2.0;
    HubShot.Solution shot = HubShot.solve(moving);
    assertTrue(shot.aim.getRadians() < 0.0);
    assertEquals(Math.PI, east.getRadians(), 1e-9);
  }

  @Test
  public void chassisHeadingRefusesAMissingPose() {
    assertNull(HubShot.chassisHeading(null, new Translation2d(4.0, 0.0)));
    assertNull(
        HubShot.chassisHeading(new Translation2d(Double.NaN, 0.0), new Translation2d(4.0, 0.0)));
    assertNull(HubShot.chassisHeading(new Translation2d(1.0, 1.0), new Translation2d(1.0, 1.0)));
  }

  @Test
  public void velocityFilterIgnoresSameCycleAndNaN() {
    HubShot.VelocityFilter filter = new HubShot.VelocityFilter();
    assertEquals(0.0, filter.update(0.0, 0.0, 0.0, 0.05).getX(), 1e-9);
    double alpha = 1.0 - Math.exp(-0.02 / 0.05);
    Translation2d filtered = filter.update(4.0, 0.0, 0.02, 0.05);
    assertEquals(4.0 * alpha, filtered.getX(), 1e-9);
    // Same timestamp must not apply the sample a second time.
    assertEquals(filtered.getX(), filter.update(0.0, 0.0, 0.02, 0.05).getX(), 1e-9);
    assertEquals(filtered.getX(), filter.update(Double.NaN, 0.0, 0.04, 0.05).getX(), 1e-9);
    // A long gap reseeds and clears acceleration.
    assertEquals(1.0, filter.update(1.0, 0.0, 1.0, 0.05).getX(), 1e-9);
    assertEquals(0.0, filter.acceleration().getNorm(), 1e-9);
  }
}
