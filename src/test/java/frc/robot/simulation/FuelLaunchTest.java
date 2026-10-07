package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Checks that sim fuel leaves from the same muzzle constants HubShot uses. */
public class FuelLaunchTest {

  /** NetworkTables inside Gamepiece needs a HAL. */
  @BeforeAll
  public static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  public void launchUsesSharedMuzzleAndFieldSpeed() {
    assertEquals(-0.183302, Constants.Shooter.kShooterForwardMeters, 0.0);
    assertEquals(70.0, Constants.Shooter.kHoodPitchDegrees, 0.0);

    Pose2d[] pose = {new Pose2d(2.0, 3.0, new Rotation2d())};
    ChassisSpeeds[] speeds = {new ChassisSpeeds()};
    Gamepiece sim = new Gamepiece("/Fuel Simulation Test");
    sim.registerRobot(
        Meters.of(0.9), Meters.of(0.9), Meters.of(0.1), () -> pose[0], () -> speeds[0]);

    sim.launchFuel(MetersPerSecond.of(10.0));
    Translation3d pos = sim.fuels.get(sim.fuels.size() - 1).pos;
    assertEquals(2.0 + Constants.Shooter.kShooterForwardMeters, pos.getX(), 1e-9);
    assertEquals(Constants.Shooter.kShooterHeightMeters, pos.getZ(), 1e-9);
    assertTrue(Math.abs(pos.getY() - 3.0) <= 0.31 - 0.075 + 1e-9);

    sim.clearFuel();
    pose[0] = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90.0));
    sim.launchFuel(MetersPerSecond.of(10.0));
    pos = sim.fuels.get(sim.fuels.size() - 1).pos;
    assertEquals(3.0 + Constants.Shooter.kShooterForwardMeters, pos.getY(), 1e-9);
    assertEquals(Constants.Shooter.kShooterHeightMeters, pos.getZ(), 1e-9);

    sim.clearFuel();
    pose[0] = new Pose2d(2.0, 3.0, new Rotation2d());
    speeds[0] = new ChassisSpeeds(5.0, 0.0, 0.0);
    sim.launchFuel(MetersPerSecond.of(10.0));
    Translation3d vel = sim.fuels.get(sim.fuels.size() - 1).vel;
    assertTrue(vel.getX() > 1.0);
  }
}
