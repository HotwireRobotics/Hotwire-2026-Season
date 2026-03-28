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

  private static final Distance DEFAULT_ROBOT_WIDTH = Inches.of(35);
  private static final Distance DEFAULT_ROBOT_LENGTH = Inches.of(35);
  private static final Distance DEFAULT_BUMPER_HEIGHT = Inches.of(4);

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
          new RobotCollisionPhysics(DEFAULT_ROBOT_WIDTH, DEFAULT_ROBOT_LENGTH, DEFAULT_BUMPER_HEIGHT);
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

    private RobotCollisionPhysics(Distance robotWidth, Distance robotLength, Distance bumperHeight) {
      this.robotWidthMeters = robotWidth.in(edu.wpi.first.units.Units.Meters);
      this.robotLengthMeters = robotLength.in(edu.wpi.first.units.Units.Meters);
      this.bumperHeightMeters = bumperHeight.in(edu.wpi.first.units.Units.Meters);
    }

    /**
     * Enforces field boundary collisions by clamping robot center based on oriented robot extents.
     */
    private void resolveFieldBoundaryCollision(
        Pose2d pose, ChassisSpeeds speeds, Consumer<Pose2d> poseSetter) {
      double halfLength = robotLengthMeters / 2.0;
      double halfWidth = robotWidthMeters / 2.0;
      double heading = pose.getRotation().getRadians();

      // Project oriented half extents into field X/Y axes for an AABB-safe boundary clamp.
      double projectedHalfX = Math.abs(Math.cos(heading)) * halfLength + Math.abs(Math.sin(heading)) * halfWidth;
      double projectedHalfY = Math.abs(Math.sin(heading)) * halfLength + Math.abs(Math.cos(heading)) * halfWidth;

      double clampedX = MathUtil.clamp(pose.getX(), projectedHalfX, FIELD_LENGTH_METERS - projectedHalfX);
      double clampedY = MathUtil.clamp(pose.getY(), projectedHalfY, FIELD_WIDTH_METERS - projectedHalfY);

      boolean hitXWall = Math.abs(clampedX - pose.getX()) > 1e-6;
      boolean hitYWall = Math.abs(clampedY - pose.getY()) > 1e-6;
      if (!hitXWall && !hitYWall) {
        return;
      }

      Pose2d correctedPose = new Pose2d(clampedX, clampedY, pose.getRotation());
      poseSetter.accept(correctedPose);

      Logger.recordOutput("Simulation/RobotCollision/HitWallX", hitXWall);
      Logger.recordOutput("Simulation/RobotCollision/HitWallY", hitYWall);
      Logger.recordOutput("Simulation/RobotCollision/BumperHeightMeters", bumperHeightMeters);
      Logger.recordOutput(
          "Simulation/RobotCollision/ImpactSpeedMps",
          Math.hypot(
              hitXWall ? speeds.vxMetersPerSecond : 0.0,
              hitYWall ? speeds.vyMetersPerSecond : 0.0));
    }
  }
}
