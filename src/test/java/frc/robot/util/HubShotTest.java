package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

/** Pure checks for moving-shot aim and RPM. These do not start the robot. */
public class HubShotTest {

  private static HubShot.Input still() {
    HubShot.Input in = new HubShot.Input();
    in.robot = new Translation2d(0.0, 0.0);
    in.hub = new Translation2d(4.0, 0.0);
    in.lookaheadSeconds = 0.0;
    in.leadGainRadiansPerMps = 0.0;
    return in;
  }

  private static double curve(double meters) {
    return 1350.92838 * Math.pow(1.00529, meters * 39.37007874015748);
  }

  @Test
  public void stationaryMatchesRegression() {
    HubShot.Solution shot = HubShot.solve(still());
    assertTrue(shot.live);
    assertEquals(0.0, shot.aim.getRadians(), 1e-9);
    assertEquals(0.0, shot.leadRadians, 1e-9);
    assertEquals(curve(4.0), shot.stationaryRpm, 1e-6);
    assertEquals(shot.stationaryRpm, shot.rpm, 1e-6);
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
    assertTrue(shot.aim.getRadians() < 0.0);
    assertTrue(shot.perpMetersPerSecond > 0.0);
    assertTrue(shot.rpm > parked.rpm);
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
  public void hugeSidewaysSpeedIsClamped() {
    HubShot.Input in = still();
    in.vyMetersPerSecond = 50.0;
    in.maxFieldSpeed = 50.0;
    in.metersPerSecondPerRpm = 0.001;
    HubShot.Solution shot = HubShot.solve(in);
    assertTrue(shot.live);
    assertTrue(Math.abs(shot.leadRadians) <= Math.toRadians(25.0) + 1e-9);
    assertTrue(shot.rpm <= 5500.0);
    assertTrue(shot.rpm <= shot.stationaryRpm * 1.40 + 1e-6);
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
    assertEquals(0.30, shot.distanceMeters, 1e-9);
    assertTrue(Double.isFinite(shot.rpm));
    assertTrue(shot.rpm > 0.0);
  }
}
