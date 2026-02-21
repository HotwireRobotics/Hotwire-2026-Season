package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import java.util.function.Supplier;

/** Calculates shooter velocity targets from robot pose and selected aiming target. */
public class ShootingVelocityCalculator {
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<Pose2d> targetPoseSupplier;

  public ShootingVelocityCalculator(
      Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> targetPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.targetPoseSupplier = targetPoseSupplier;
  }

  /** Computes velocity from distance regression, returning zero when pose data is unavailable. */
  public AngularVelocity regressionVelocity() {
    Pose2d currentPose = robotPoseSupplier.get();
    Pose2d targetPose = targetPoseSupplier.get();
    if (currentPose == null || targetPose == null) {
      return RPM.of(0.0);
    }
    return Constants.regress(Meters.of(currentPose.minus(targetPose).getTranslation().getNorm()));
  }
}
