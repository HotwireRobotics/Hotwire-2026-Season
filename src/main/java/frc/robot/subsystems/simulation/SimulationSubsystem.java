package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Mode;
import frc.robot.simulation.Handler;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Isolates simulation-only behavior and robot physics from runtime logic. */
public class SimulationSubsystem extends SubsystemBase {
  private static final double FIELD_LENGTH_METERS = 16.51;
  private static final double FIELD_WIDTH_METERS = 8.04;

  private static final Distance ROBOT_WIDTH_WITH_BUMPERS = Inches.of(34);
  private static final Distance ROBOT_LENGTH_WITH_BUMPERS = Inches.of(34);
  private static final Distance BUMPER_HEIGHT = Inches.of(5);
  private static final Distance BUMPER_CLEARANCE = Inches.of(2.5);
  private static final Distance BUMPER_SQUISH_COMPLIANCE = Inches.of(0.25);
  private static final double ROBOT_MASS_KG = 105.0 * 0.45359237;

  private final Mode mode;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final Consumer<Pose2d> poseSetter;
  private final Handler simulationHandler;
  private final RobotCollisionPhysics collisionPhysics;

  /**
   * Creates the simulation subsystem with runtime-safe no-op behavior outside SIM mode.
   */
  public SimulationSubsystem(
      Mode mode,
      Supplier<AngularVelocity> shooterVelocitySupplier,
      BooleanSupplier isShootingSupplier,
      BooleanSupplier isIntakingSupplier,
      Supplier<Angle> intakeWristSupplier,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Consumer<Pose2d> poseSetter) {
    this.mode = mode;
    this.poseSupplier = poseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.poseSetter = poseSetter;

    if (mode == Mode.SIM) {
      simulationHandler =
          new Handler(
              shooterVelocitySupplier,
              isShootingSupplier,
              isIntakingSupplier,
              intakeWristSupplier,
              poseSupplier,
              chassisSpeedsSupplier);
      collisionPhysics =
          new RobotCollisionPhysics(
              ROBOT_WIDTH_WITH_BUMPERS,
              ROBOT_LENGTH_WITH_BUMPERS,
              BUMPER_HEIGHT,
              BUMPER_CLEARANCE,
              BUMPER_SQUISH_COMPLIANCE,
              ROBOT_MASS_KG);
    } else {
      simulationHandler = null;
      collisionPhysics = null;
    }
  }

  /** Updates simulation internals and robot model logging for the active mode. */
  public void tick() {
    Logger.recordOutput("Simulation/RobotModel", mode == Mode.SIM ? "robot3d" : "robot2d");
    if (mode != Mode.SIM || simulationHandler == null || collisionPhysics == null) {
      logStaticRobotModel();
      return;
    }

    collisionPhysics.resolveFieldBoundaryCollision(poseSupplier.get(), chassisSpeedsSupplier.get(), poseSetter);
    simulationHandler.tick();
    logAnimatedRobotModel(simulationHandler.getWristPitch());
  }

  /** Resets simulation state when autonomous starts. */
  public void autonomous() {
    if (simulationHandler != null) {
      simulationHandler.autonomous();
    }
  }

  private void logStaticRobotModel() {
    Logger.recordOutput("RobotPose", poseSupplier.get());
    Logger.recordOutput("Simulation/RobotPose3d", new Pose3d(poseSupplier.get()));
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {});
    Logger.recordOutput("FinalComponentPoses", new Pose3d[] {});
  }

  private void logAnimatedRobotModel(Angle wristPitch) {
    Pose3d robotPose3d = new Pose3d(poseSupplier.get());
    Logger.recordOutput("RobotPose", poseSupplier.get());
    Logger.recordOutput("Simulation/RobotPose3d", robotPose3d);
    Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          new Pose3d(
              0.1958,
              0.0,
              0.21,
              new Rotation3d(
                  Rotations.of(0),
                  wristPitch,
                  Rotations.of(0)))
        });
  }

  /** Lightweight collision model to keep the robot physically inside the field in simulation. */
  private static class RobotCollisionPhysics {
    private final double robotWidthMeters;
    private final double robotLengthMeters;
    private final double bumperHeightMeters;
    private final double bumperClearanceMeters;
    private final double bumperComplianceMeters;
    private final double robotMassKg;
    private final double coefficientOfRestitution;
    private final double tangentFrictionCoefficient;

    private RobotCollisionPhysics(
        Distance robotWidth,
        Distance robotLength,
        Distance bumperHeight,
        Distance bumperClearance,
        Distance bumperCompliance,
        double robotMassKg) {
      this.robotWidthMeters = robotWidth.in(edu.wpi.first.units.Units.Meters);
      this.robotLengthMeters = robotLength.in(edu.wpi.first.units.Units.Meters);
      this.bumperHeightMeters = bumperHeight.in(edu.wpi.first.units.Units.Meters);
      this.bumperClearanceMeters = bumperClearance.in(edu.wpi.first.units.Units.Meters);
      this.bumperComplianceMeters = bumperCompliance.in(edu.wpi.first.units.Units.Meters);
      this.robotMassKg = robotMassKg;
      // Slight bumper squish: mostly inelastic with little bounce.
      this.coefficientOfRestitution = 0.12;
      this.tangentFrictionCoefficient = 0.65;
    }

    /**
     * Enforces collisions with field boundaries and hub keep-out zones.
     */
    private void resolveFieldBoundaryCollision(
        Pose2d pose, ChassisSpeeds speeds, Consumer<Pose2d> poseSetter) {
      double halfLength = robotLengthMeters / 2.0;
      double halfWidth = robotWidthMeters / 2.0;
      double heading = pose.getRotation().getRadians();

      // Project oriented half extents into field X/Y axes for an AABB-safe boundary clamp.
      double projectedHalfX = Math.abs(Math.cos(heading)) * halfLength + Math.abs(Math.sin(heading)) * halfWidth;
      double projectedHalfY = Math.abs(Math.sin(heading)) * halfLength + Math.abs(Math.cos(heading)) * halfWidth;
      double complianceX = Math.min(projectedHalfX * 0.4, bumperComplianceMeters);
      double complianceY = Math.min(projectedHalfY * 0.4, bumperComplianceMeters);

      double clampedX =
          MathUtil.clamp(
              pose.getX(),
              projectedHalfX - complianceX,
              FIELD_LENGTH_METERS - projectedHalfX + complianceX);
      double clampedY =
          MathUtil.clamp(
              pose.getY(),
              projectedHalfY - complianceY,
              FIELD_WIDTH_METERS - projectedHalfY + complianceY);

      boolean hitXWall = Math.abs(clampedX - pose.getX()) > 1e-6;
      boolean hitYWall = Math.abs(clampedY - pose.getY()) > 1e-6;
      Pose2d correctedPose = new Pose2d(clampedX, clampedY, pose.getRotation());

      // Hub keep-out colliders (center + estimated perimeter radius from 2020 field model).
      correctedPose = resolveHubCollider(correctedPose, 4.61, FIELD_WIDTH_METERS / 2.0, 0.6, halfLength, halfWidth);
      correctedPose =
          resolveHubCollider(
              correctedPose,
              FIELD_LENGTH_METERS - 4.61,
              FIELD_WIDTH_METERS / 2.0,
              0.6,
              halfLength,
              halfWidth);

      boolean correctedByCollider = correctedPose.getTranslation().getDistance(pose.getTranslation()) > 1e-6;
      if (hitXWall || hitYWall || correctedByCollider) {
        poseSetter.accept(correctedPose);
      }

      double normalImpactSpeed =
          Math.hypot(hitXWall ? speeds.vxMetersPerSecond : 0.0, hitYWall ? speeds.vyMetersPerSecond : 0.0);
      double normalImpulseNewtonSeconds =
          robotMassKg * (1.0 + coefficientOfRestitution) * normalImpactSpeed;
      double tangentImpactSpeed =
          Math.hypot(hitYWall ? speeds.vxMetersPerSecond : 0.0, hitXWall ? speeds.vyMetersPerSecond : 0.0);
      double frictionImpulseNewtonSeconds =
          robotMassKg * tangentFrictionCoefficient * tangentImpactSpeed;

      Logger.recordOutput("Simulation/RobotCollision/HitWallX", hitXWall || correctedByCollider);
      Logger.recordOutput("Simulation/RobotCollision/HitWallY", hitYWall || correctedByCollider);
      Logger.recordOutput("Simulation/RobotCollision/BumperHeightMeters", bumperHeightMeters);
      Logger.recordOutput("Simulation/RobotCollision/BumperClearanceMeters", bumperClearanceMeters);
      Logger.recordOutput("Simulation/RobotCollision/BumperComplianceMeters", bumperComplianceMeters);
      Logger.recordOutput("Simulation/RobotCollision/MassKg", robotMassKg);
      Logger.recordOutput("Simulation/RobotCollision/Restitution", coefficientOfRestitution);
      Logger.recordOutput(
          "Simulation/RobotCollision/ImpactSpeedMps",
          normalImpactSpeed);
      Logger.recordOutput(
          "Simulation/RobotCollision/NormalImpulseNs",
          normalImpulseNewtonSeconds);
      Logger.recordOutput(
          "Simulation/RobotCollision/FrictionImpulseNs",
          frictionImpulseNewtonSeconds);
    }

    /**
     * Resolves a simple circular keep-out collider around major centerfield structures.
     */
    private Pose2d resolveHubCollider(
        Pose2d pose, double centerX, double centerY, double hubRadiusMeters, double halfLength, double halfWidth) {
      double robotRadius = Math.hypot(halfLength, halfWidth);
      double dx = pose.getX() - centerX;
      double dy = pose.getY() - centerY;
      double distance = Math.hypot(dx, dy);
      double minimumDistance = hubRadiusMeters + robotRadius - bumperComplianceMeters;

      if (distance >= minimumDistance || distance < 1e-9) {
        if (distance < 1e-9) {
          return new Pose2d(centerX + minimumDistance, centerY, pose.getRotation());
        }
        return pose;
      }

      double scale = minimumDistance / distance;
      return new Pose2d(centerX + dx * scale, centerY + dy * scale, pose.getRotation());
    }
  }
}
