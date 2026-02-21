package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

/** Unit tests for shooter velocity target calculation logic. */
class ShootingVelocityCalculatorTest {
  @Test
  void returnsZeroWhenSuppliersHaveNullPose() {
    ShootingVelocityCalculator calculator = new ShootingVelocityCalculator(() -> null, () -> null);
    assertEquals(0.0, calculator.regressionVelocity().in(RPM), 1e-9);
  }

  @Test
  void returnsRegressionVelocityForKnownDistance() {
    Pose2d robotPose = new Pose2d(Meters.of(4.0), Meters.of(3.0), Rotation2d.kZero);
    Pose2d targetPose = new Pose2d(Meters.of(1.0), Meters.of(3.0), Rotation2d.kZero);
    ShootingVelocityCalculator calculator =
        new ShootingVelocityCalculator(() -> robotPose, () -> targetPose);

    double expectedRpm = Constants.regress(Meters.of(3.0)).in(RPM);
    assertEquals(expectedRpm, calculator.regressionVelocity().in(RPM), 1e-9);
  }
}
