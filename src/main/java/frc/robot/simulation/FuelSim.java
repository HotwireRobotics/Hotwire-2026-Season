package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Fuel physics for simulation. Balls leave through {@link Gamepiece#launchFuel(LinearVelocity)},
 * which reads the same muzzle constants as {@code HubShot}.
 */
public final class FuelSim {
  /** Hopper size copied from the hotwire-simulation Handler. */
  private static final int HOPPER_LIMIT = 28;

  /**
   * Seconds between launches while the flywheel is on. The old Handler aimed at about four fuel
   * per second.
   */
  private static final double SECONDS_PER_FUEL = 0.25;

  /** Wheel radius used by the old Handler to turn RPM into exit speed. */
  private static final Distance WHEEL_RADIUS = Inches.of(1.2);

  /** Fuel in the robot at sim start, so a teleop shot can be seen before intaking. */
  private static final int STARTING_HELD = 8;

  private final Gamepiece gamepiece;
  private final BooleanSupplier firing;
  private final Supplier<AngularVelocity> flywheel;
  private int held = STARTING_HELD;
  private double sinceLaunch;

  /**
   * Spawn the field, register this robot, and start physics.
   *
   * @param pose odometry pose
   * @param fieldSpeeds field-relative chassis velocity added to each launch
   * @param firing true while {@code Shooter} is in the firing state
   * @param intaking true while the intake rollers are running
   * @param flywheel commanded shooter speed; shoot-on-the-fly already returns the live RPM
   */
  public FuelSim(
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> fieldSpeeds,
      BooleanSupplier firing,
      BooleanSupplier intaking,
      Supplier<AngularVelocity> flywheel) {
    this.firing = firing;
    this.flywheel = flywheel;

    gamepiece = new Gamepiece();
    gamepiece.spawnStartingFuel();
    gamepiece.registerRobot(Inches.of(35), Inches.of(35), Inches.of(4), pose, fieldSpeeds);
    // Intake window from the hotwire-simulation Handler, robot frame.
    gamepiece.registerIntake(
        Inches.of(17.5),
        Inches.of(24.118),
        Inches.of(-14.5),
        Inches.of(15.5),
        () -> intaking.getAsBoolean() && held < HOPPER_LIMIT,
        this::intake);
    gamepiece.setSubticks(5);
    gamepiece.setLoggingFrequency(30);
    gamepiece.enableAirResistance();
    gamepiece.start();
  }

  /** Step fuel physics and launch while the flywheel is running and the hopper is not empty. */
  public void tick() {
    if (firing.getAsBoolean()) {
      sinceLaunch += Gamepiece.PERIOD;
      if (held > 0 && sinceLaunch >= SECONDS_PER_FUEL) {
        sinceLaunch = 0.0;
        launch();
      }
    } else {
      sinceLaunch = 0.0;
    }
    gamepiece.updateSim();
    Logger.recordOutput("Simulation/Hopper", held);
    Logger.recordOutput("Simulation/Score/Blue", Gamepiece.Hub.BLUE_HUB.getScore());
    Logger.recordOutput("Simulation/Score/Red", Gamepiece.Hub.RED_HUB.getScore());
  }

  /** Reset the field and hopper the way the old sim did at the start of auto. */
  public void autonomous() {
    gamepiece.clearFuel();
    gamepiece.spawnStartingFuel();
    Gamepiece.Hub.BLUE_HUB.resetScore();
    Gamepiece.Hub.RED_HUB.resetScore();
    held = STARTING_HELD;
    sinceLaunch = 0.0;
  }

  /** One ball entered the robot. */
  private void intake() {
    if (held < HOPPER_LIMIT) {
      held++;
    }
  }

  /** One ball leaves the muzzle at the flywheel's tangential speed. */
  private void launch() {
    AngularVelocity speed = flywheel.get();
    if (speed == null || !Double.isFinite(speed.in(RotationsPerSecond))) {
      return;
    }
    gamepiece.launchFuel(exitSpeed(speed));
    held--;
  }

  /**
   * Tangential speed of a roller with {@link #WHEEL_RADIUS}. Same conversion the old Handler
   * called {@code lineate}.
   */
  private static LinearVelocity exitSpeed(AngularVelocity speed) {
    return WHEEL_RADIUS.times(Constants.Mathematics.TAU)
        .per(Second)
        .times(speed.in(RotationsPerSecond));
  }
}
