package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.Constants;

public class Handler {
    private static final double FIELD_LENGTH_METERS = 16.51;
    private static final double FIELD_WIDTH_METERS = 8.04;

    private static final Distance ROBOT_WIDTH_WITH_BUMPERS = Inches.of(34);
    private static final Distance ROBOT_LENGTH_WITH_BUMPERS = Inches.of(34);
    private static final Distance BUMPER_HEIGHT = Inches.of(5);
    private static final Distance BUMPER_CLEARANCE = Inches.of(2.5);
    private static final Distance BUMPER_SQUISH_COMPLIANCE = Inches.of(0.25);
    private static final double ROBOT_MASS_KG = 105.0 * 0.45359237;

    // Hopper count.
    private int counter = 0;
    private Angle motion = Degrees.of(0);
    private Angle pitch = Degrees.of(0);
    private final int limit = 28;
    // Declare supplier for shooting.
    private final Supplier<AngularVelocity> velocity;
    private final BooleanSupplier doShoot;
    private final BooleanSupplier doIntake;
    private final Supplier<Angle> target;

    // Drive suppliers.
    private final Supplier<Pose2d> pose;
    private final Consumer<Pose2d> supp;
    private final Supplier<ChassisSpeeds> chassisSpeeds;

    private final Gamepiece gamepieceSimulation;

    private final RobotCollisionPhysics physics;
    
    public Handler(
        Supplier<AngularVelocity> velocity,
        BooleanSupplier shooter,
        BooleanSupplier intake,
        Supplier<Angle> wrist,
        Supplier<Pose2d> pose,
        Supplier<ChassisSpeeds> chassisSpeeds,
        Consumer<Pose2d> supp
    ) {
        this.velocity = velocity;

        this.supp = supp;

        doIntake = () -> {
            return intake.getAsBoolean() && (counter < limit) && (Math.random() > 0.99) && (pitch.lte(Degrees.of(3)));
        };

        target = wrist;

        doShoot = shooter;

        this.pose = pose;
        this.chassisSpeeds = chassisSpeeds;

        gamepieceSimulation = new Gamepiece();
        gamepieceSimulation.spawnStartingFuel();

        physics = new RobotCollisionPhysics(
            ROBOT_WIDTH_WITH_BUMPERS,
            ROBOT_LENGTH_WITH_BUMPERS,
            BUMPER_HEIGHT,
            BUMPER_CLEARANCE,
            BUMPER_SQUISH_COMPLIANCE,
            ROBOT_MASS_KG
        );

        // Register a robot for collision with fuel.
        gamepieceSimulation.registerRobot(
                Inches.of(35),
                Inches.of(35),
                Inches.of(4),
                this.pose, this.chassisSpeeds);

        gamepieceSimulation.registerIntake(
            Inches.of(17.5), Inches.of(24.118), Inches.of(-14.5), Inches.of(15.5), doIntake, this::intake);
        
        gamepieceSimulation.setSubticks(5);
        gamepieceSimulation.setLoggingFrequency(30);
        gamepieceSimulation.enableAirResistance();
        gamepieceSimulation.start();
    }

    /** Attempt to decrement the gamepiece counter. */
    private void shoot() {
        // Random chance of not firing based on the fact that we usually only shoot ~4 per second.
        Time time = Constants.Tempo.getTime();
        if (
            ((counter > 0) && ((time.in(Seconds) % ((10 / ((-50 * motion.in(Degrees)) + (3 * counter))))) + (Math.random()/10)) < 0.05)
        ) {
            gamepieceSimulation.launchFuel(lineate(velocity.get(), Inches.of(1.2)));
            counter --;
        }
    }

    /** Attempt to increment gamepiece counter. */
    public void intake() {
        this.counter ++;
    }

    /** Initialize with gamepiece(s). */
    public void setCounter(
        int count
    ) {
        counter = count;             
    }

    /** Update simulation. */
    public void tick() {
        if (doShoot.getAsBoolean()) this.shoot();
        gamepieceSimulation.updateSim();

        Logger.recordOutput("Simulation/Score/Blue", Gamepiece.Hub.BLUE_HUB.getScore());
        Logger.recordOutput("Simulation/Score/Red",  Gamepiece.Hub.RED_HUB.getScore());
        Logger.recordOutput("Simulation/Pitch", pitch);
        Logger.recordOutput("Simulation/Motion", motion);

        motion = (pitch.minus(target.get().times(-1))).times(0.1).plus(
            (pitch.gt(Degrees.of(0)) ? Degrees.of(Math.random() * 0.03) : Degrees.of(0)));
        
        pitch = pitch.minus(motion);
        
        Pose3d robotPose3d = physics.getRobotPose3d(pose.get());
        Logger.recordOutput("Simulation/RobotPose3d", robotPose3d);
        Logger.recordOutput("RobotPose", pose.get());
        Logger.recordOutput("ZeroedComponentPoses", new Pose3d[] {new Pose3d()});
        Logger.recordOutput("Intake", new Pose3d[] {
            new Pose3d(
                0.1958, 0.0, 0.21, 
                new Rotation3d(
                    Rotations.of(0), 
                    getWristPitch(), 
                    Rotations.of(0)
                ))
        });
        physics.resolveFieldBoundaryCollision(pose.get(), chassisSpeeds.get(), supp);
    }

    public void restart() {
        gamepieceSimulation.clearFuel();
        gamepieceSimulation.spawnStartingFuel(); 

        Gamepiece.Hub.BLUE_HUB.resetScore();
        Gamepiece.Hub.RED_HUB.resetScore();
    }

    public void autonomous() {
        restart();
        setCounter(8);
    }

    public Angle getWristPitch() {
        return pitch;
    }

    private LinearVelocity lineate(AngularVelocity velocity, Distance radius) {
        return radius.times(Constants.Mathematics.TAU).per(Second).times(velocity.in(RotationsPerSecond));
    }

  /** Lightweight collision model to keep the robot physically inside the field in simulation. */
  private static class RobotCollisionPhysics {
    private static final double SIM_DT_SECONDS = 0.02;
    private static final double MAX_LINEAR_ACCEL_MPS2 = 3.2;
    private static final double MAX_ANGULAR_ACCEL_RADPS2 = 7.5;
    private static final double MAX_ROBOT_TILT_RAD = Math.toRadians(24);
    private static final double TERRAIN_PITCH_SIGN = -1.0;
    private static final double TERRAIN_ROLL_SIGN = 1.0;
    private static final double WHEEL_CONTACT_HEIGHT_TOLERANCE_METERS = 0.01;
    private static final double CHASSIS_CONTACT_EPSILON_METERS = 0.0008;
    private static final double GRAVITY_MPS2 = 7.1;
    private static final double AIR_TILT_STIFFNESS = 18.0;
    private static final double AIR_TILT_DAMPING = 0.65;
    private static final double SUPPORT_LAUNCH_VELOCITY_GAIN = 1.45;
    private static final double MAX_SUPPORT_LAUNCH_VELOCITY_MPS = 2.8;
    private static final double TERRAIN_PITCH_RESPONSE_GAIN = 1.35;
    private static final double TERRAIN_ROLL_RESPONSE_GAIN = 1.9;
    private static final double COLLISION_FRICTION_COEFFICIENT = 0.55;
    private static final double CENTER_OF_MASS_HEIGHT_METERS = 0.28;
    private static final double ROBOT_BODY_HEIGHT_METERS = 0.30;
    private static final double MAX_COLLISION_TILT_RATE_RADPS = Math.toRadians(1200);
    private static final double MAX_COLLISION_YAW_STEP_RAD = Math.toRadians(28.0);
    private static final double HUB_SIDE = 1.2;
    private static final double BUMP_ENTRY_X = 3.96;
    private static final double BUMP_PEAK_X = 4.61;
    private static final double BUMP_EXIT_X = 5.18;
    private static final double BUMP_HEIGHT = 0.165;
    private static final double BUMP_LOW_Y_MIN = 1.57;
    private static final double BUMP_LOW_Y_MAX = FIELD_WIDTH_METERS / 2.0 - 0.60;
    private static final double BUMP_HIGH_Y_MIN = FIELD_WIDTH_METERS / 2.0 + 0.60;
    private static final double BUMP_HIGH_Y_MAX = FIELD_WIDTH_METERS - 1.57;
    private static final double TRENCH_WIDTH = 1.265;
    private static final double TRENCH_BLOCK_WIDTH = 0.305;
    private static final double TRENCH_BAR_WIDTH = 0.152;

    private final ColliderRect[] staticRectangles = {
      // Hub side walls.
      new ColliderRect(4.61 - HUB_SIDE / 2, FIELD_WIDTH_METERS / 2 - HUB_SIDE / 2, 4.61 + HUB_SIDE / 2, FIELD_WIDTH_METERS / 2 + HUB_SIDE / 2),
      new ColliderRect(
          FIELD_LENGTH_METERS - 4.61 - HUB_SIDE / 2,
          FIELD_WIDTH_METERS / 2 - HUB_SIDE / 2,
          FIELD_LENGTH_METERS - 4.61 + HUB_SIDE / 2,
          FIELD_WIDTH_METERS / 2 + HUB_SIDE / 2),
      // Trench blocks.
      new ColliderRect(3.96, TRENCH_WIDTH, 5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH),
      new ColliderRect(3.96, FIELD_WIDTH_METERS - 1.57, 5.18, FIELD_WIDTH_METERS - 1.57 + TRENCH_BLOCK_WIDTH),
      new ColliderRect(FIELD_LENGTH_METERS - 5.18, TRENCH_WIDTH, FIELD_LENGTH_METERS - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH),
      new ColliderRect(
          FIELD_LENGTH_METERS - 5.18,
          FIELD_WIDTH_METERS - 1.57,
          FIELD_LENGTH_METERS - 3.96,
          FIELD_WIDTH_METERS - 1.57 + TRENCH_BLOCK_WIDTH),
      // Trench bars.
      new ColliderRect(
          4.61 - TRENCH_BAR_WIDTH / 2.0,
          0.0,
          4.61 + TRENCH_BAR_WIDTH / 2.0,
          TRENCH_WIDTH + TRENCH_BLOCK_WIDTH),
      new ColliderRect(
          4.61 - TRENCH_BAR_WIDTH / 2.0,
          FIELD_WIDTH_METERS - 1.57,
          4.61 + TRENCH_BAR_WIDTH / 2.0,
          FIELD_WIDTH_METERS),
      new ColliderRect(
          FIELD_LENGTH_METERS - 4.61 - TRENCH_BAR_WIDTH / 2.0,
          0.0,
          FIELD_LENGTH_METERS - 4.61 + TRENCH_BAR_WIDTH / 2.0,
          TRENCH_WIDTH + TRENCH_BLOCK_WIDTH),
      new ColliderRect(
          FIELD_LENGTH_METERS - 4.61 - TRENCH_BAR_WIDTH / 2.0,
          FIELD_WIDTH_METERS - 1.57,
          FIELD_LENGTH_METERS - 4.61 + TRENCH_BAR_WIDTH / 2.0,
          FIELD_WIDTH_METERS)
    };

    private final double robotWidthMeters;
    private final double robotLengthMeters;
    private final double bumperHeightMeters;
    private final double bumperClearanceMeters;
    private final double bumperComplianceMeters;
    private final double robotMassKg;
    private final double coefficientOfRestitution;
    private final double tangentFrictionCoefficient;
    private ChassisSpeeds previousSpeeds = new ChassisSpeeds();
    private double pitchRad = 0.0;
    private double rollRad = 0.0;
    private double pitchRateRadPerSec = 0.0;
    private double rollRateRadPerSec = 0.0;
    private double chassisHeightMeters = 0.0;
    private double chassisVerticalVelocityMps = 0.0;
    private boolean allWheelsGrounded = true;
    private boolean chassisAirborne = false;
    private double previousSupportHeightMeters = 0.0;
    private Translation2d lastCollisionNormal = null;

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
      double clampedX =
          MathUtil.clamp(
              pose.getX(),
              projectedHalfX,
              FIELD_LENGTH_METERS - projectedHalfX);
      double clampedY =
          MathUtil.clamp(
              pose.getY(),
              projectedHalfY,
              FIELD_WIDTH_METERS - projectedHalfY);

      boolean hitXWall = Math.abs(clampedX - pose.getX()) > 1e-6;
      boolean hitYWall = Math.abs(clampedY - pose.getY()) > 1e-6;
      Pose2d correctedPose = new Pose2d(clampedX, clampedY, pose.getRotation());
      correctedPose = resolveStaticColliders(correctedPose, halfLength, halfWidth, 20);

      boolean correctedByCollider = correctedPose.getTranslation().getDistance(pose.getTranslation()) > 1e-6;
      applyCollisionTiltResponse(pose, correctedPose, speeds);
      correctedPose = applyCollisionYawResponse(pose, correctedPose, speeds);
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
      double linearAccelerationMps2 =
          Math.hypot(
                  speeds.vxMetersPerSecond - previousSpeeds.vxMetersPerSecond,
                  speeds.vyMetersPerSecond - previousSpeeds.vyMetersPerSecond)
              / SIM_DT_SECONDS;
      double angularAccelerationRadps2 =
          Math.abs(speeds.omegaRadiansPerSecond - previousSpeeds.omegaRadiansPerSecond) / SIM_DT_SECONDS;
      double ax = (speeds.vxMetersPerSecond - previousSpeeds.vxMetersPerSecond) / SIM_DT_SECONDS;
      double ay = (speeds.vyMetersPerSecond - previousSpeeds.vyMetersPerSecond) / SIM_DT_SECONDS;
      updateRobotTilt(correctedPose, ax, ay);
      previousSpeeds = speeds;

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
      Logger.recordOutput(
          "Simulation/RobotPhysics/LinearAccelerationMps2",
          Math.min(linearAccelerationMps2, MAX_LINEAR_ACCEL_MPS2));
      Logger.recordOutput(
          "Simulation/RobotPhysics/AngularAccelerationRadps2",
          Math.min(angularAccelerationRadps2, MAX_ANGULAR_ACCEL_RADPS2));
      Logger.recordOutput("Simulation/RobotPhysics/PitchDeg", Math.toDegrees(pitchRad));
      Logger.recordOutput("Simulation/RobotPhysics/RollDeg", Math.toDegrees(rollRad));
      Logger.recordOutput("Simulation/RobotPhysics/OnBump", getTerrainHeight(correctedPose.getX(), correctedPose.getY()) > 1e-3);
    }

    /**
     * Applies all static colliders currently used by fuel interactions to the robot body.
     */
    private Pose2d resolveStaticColliders(
        Pose2d pose, double halfLength, double halfWidth, int passes) {
      Pose2d corrected = pose;
      lastCollisionNormal = null;
      for (int pass = 0; pass < passes; pass++) {
        boolean changed = false;
        for (ColliderRect rect : staticRectangles) {
          CollisionResult collision = resolveRectangleCollision(corrected, rect, halfLength, halfWidth);
          if (collision != null) {
            corrected = collision.correctedPose();
            lastCollisionNormal = collision.collisionNormal();
            changed = true;
          }
        }
        if (!changed) {
          break;
        }
      }
      return corrected;
    }

    /**
     * Uses SAT-style extents in field axes for an oriented-robot vs axis-aligned-rect collision test.
     */
    private CollisionResult resolveRectangleCollision(
        Pose2d pose, ColliderRect rect, double halfLength, double halfWidth) {
      Translation2d[] robotVertices = getRobotVertices(pose, halfLength, halfWidth);
      Translation2d[] rectVertices = {
        new Translation2d(rect.xMin, rect.yMin),
        new Translation2d(rect.xMax, rect.yMin),
        new Translation2d(rect.xMax, rect.yMax),
        new Translation2d(rect.xMin, rect.yMax)
      };
      Translation2d[] axes = {
        getEdgeNormal(robotVertices[0], robotVertices[1]),
        getEdgeNormal(robotVertices[1], robotVertices[2]),
        new Translation2d(1.0, 0.0),
        new Translation2d(0.0, 1.0)
      };

      double minOverlap = Double.POSITIVE_INFINITY;
      Translation2d smallestAxis = null;
      for (Translation2d axis : axes) {
        Projection robotProjection = projectOntoAxis(robotVertices, axis);
        Projection rectProjection = projectOntoAxis(rectVertices, axis);
        double overlap = Math.min(robotProjection.max(), rectProjection.max()) - Math.max(robotProjection.min(), rectProjection.min());
        if (overlap <= 0.0) {
          return null;
        }
        if (overlap < minOverlap) {
          minOverlap = overlap;
          smallestAxis = axis;
        }
      }

      if (smallestAxis == null) {
        return null;
      }

      Translation2d centerToRect =
          new Translation2d((rect.xMin + rect.xMax) * 0.5 - pose.getX(), (rect.yMin + rect.yMax) * 0.5 - pose.getY());
      if (centerToRect.dot(smallestAxis) < 0.0) {
        smallestAxis = smallestAxis.times(-1.0);
      }

      // Add a tiny slop so the robot is immediately projected outside the static obstacle.
      Translation2d correction = smallestAxis.times(-(minOverlap + 1e-4));
      Pose2d corrected =
          new Pose2d(pose.getX() + correction.getX(), pose.getY() + correction.getY(), pose.getRotation());
      return new CollisionResult(corrected, smallestAxis, minOverlap);
    }

    /**
     * Updates pitch and roll from terrain gradient and inertial load transfer.
     */
    private void updateRobotTilt(Pose2d pose, double ax, double ay) {
      double heading = pose.getRotation().getRadians();
      double headingCos = Math.cos(heading);
      double headingSin = Math.sin(heading);
      double halfLength = robotLengthMeters / 2.0;
      double halfWidth = robotWidthMeters / 2.0;

      Translation2d frontOffset = new Translation2d(headingCos * halfLength, headingSin * halfLength);
      Translation2d sideOffset = new Translation2d(-headingSin * halfWidth, headingCos * halfWidth);
      Translation2d center = pose.getTranslation();

      double frontLeftHeight = getTerrainHeight(center.plus(frontOffset).plus(sideOffset).getX(), center.plus(frontOffset).plus(sideOffset).getY());
      double frontRightHeight = getTerrainHeight(center.plus(frontOffset).minus(sideOffset).getX(), center.plus(frontOffset).minus(sideOffset).getY());
      double rearLeftHeight = getTerrainHeight(center.minus(frontOffset).plus(sideOffset).getX(), center.minus(frontOffset).plus(sideOffset).getY());
      double rearRightHeight = getTerrainHeight(center.minus(frontOffset).minus(sideOffset).getX(), center.minus(frontOffset).minus(sideOffset).getY());

      double maxWheelHeight =
          Math.max(Math.max(frontLeftHeight, frontRightHeight), Math.max(rearLeftHeight, rearRightHeight));
      double minWheelHeight =
          Math.min(Math.min(frontLeftHeight, frontRightHeight), Math.min(rearLeftHeight, rearRightHeight));
      double wheelHeightSpread = maxWheelHeight - minWheelHeight;
      allWheelsGrounded = wheelHeightSpread <= WHEEL_CONTACT_HEIGHT_TOLERANCE_METERS;

      double frontAvg = (frontLeftHeight + frontRightHeight) * 0.5;
      double rearAvg = (rearLeftHeight + rearRightHeight) * 0.5;
      double leftAvg = (frontLeftHeight + rearLeftHeight) * 0.5;
      double rightAvg = (frontRightHeight + rearRightHeight) * 0.5;

      double terrainPitch = TERRAIN_PITCH_SIGN * Math.atan2(frontAvg - rearAvg, robotLengthMeters);
      double terrainRoll = TERRAIN_ROLL_SIGN * Math.atan2(leftAvg - rightAvg, robotWidthMeters);
      double targetHeight = Math.max(0.0, (frontAvg + rearAvg) * 0.5);
      double supportVelocityMps = (targetHeight - previousSupportHeightMeters) / SIM_DT_SECONDS;
      previousSupportHeightMeters = targetHeight;

      double longitudinalAccel = ax * headingCos + ay * headingSin;
      double lateralAccel = -ax * headingSin + ay * headingCos;

      double inertialPitch = MathUtil.clamp(-longitudinalAccel / 9.81 * 0.18, -0.16, 0.16);
      double inertialRoll = MathUtil.clamp(lateralAccel / 9.81 * 0.22, -0.20, 0.20);

      double targetPitch =
          MathUtil.clamp(
              terrainPitch * TERRAIN_PITCH_RESPONSE_GAIN + inertialPitch,
              -MAX_ROBOT_TILT_RAD,
              MAX_ROBOT_TILT_RAD);
      double targetRoll =
          MathUtil.clamp(
              terrainRoll * TERRAIN_ROLL_RESPONSE_GAIN + inertialRoll,
              -MAX_ROBOT_TILT_RAD,
              MAX_ROBOT_TILT_RAD);

      // Vertical rigid-body dynamics: gravity + moving support from terrain.
      chassisVerticalVelocityMps -= GRAVITY_MPS2 * SIM_DT_SECONDS;
      chassisHeightMeters += chassisVerticalVelocityMps * SIM_DT_SECONDS;
      if (chassisHeightMeters <= targetHeight + CHASSIS_CONTACT_EPSILON_METERS) {
        chassisHeightMeters = targetHeight;
        double launchVelocity =
            Math.min(
                supportVelocityMps * SUPPORT_LAUNCH_VELOCITY_GAIN,
                MAX_SUPPORT_LAUNCH_VELOCITY_MPS);
        if (launchVelocity > chassisVerticalVelocityMps) {
          // Preserve momentum over crest transitions so the robot can "jump".
          chassisVerticalVelocityMps = launchVelocity;
        } else if (chassisVerticalVelocityMps < 0.0) {
          chassisVerticalVelocityMps = 0.0;
        }
      }
      chassisAirborne = chassisHeightMeters > targetHeight + CHASSIS_CONTACT_EPSILON_METERS;

      if (!chassisAirborne) {
        // Ground contact is rigid: immediately align chassis to terrain support plane.
        pitchRad = MathUtil.clamp(terrainPitch, -MAX_ROBOT_TILT_RAD, MAX_ROBOT_TILT_RAD);
        rollRad = MathUtil.clamp(terrainRoll, -MAX_ROBOT_TILT_RAD, MAX_ROBOT_TILT_RAD);
        pitchRateRadPerSec = 0.0;
        rollRateRadPerSec = 0.0;
        return;
      }

      // Only airborne chassis uses dynamic tilt integration.
      double pitchAccel =
          AIR_TILT_STIFFNESS * (targetPitch - pitchRad) - AIR_TILT_DAMPING * pitchRateRadPerSec;
      double rollAccel =
          AIR_TILT_STIFFNESS * (targetRoll - rollRad) - AIR_TILT_DAMPING * rollRateRadPerSec;
      pitchRateRadPerSec += pitchAccel * SIM_DT_SECONDS;
      rollRateRadPerSec += rollAccel * SIM_DT_SECONDS;
      pitchRad =
          MathUtil.clamp(
              pitchRad + pitchRateRadPerSec * SIM_DT_SECONDS, -MAX_ROBOT_TILT_RAD, MAX_ROBOT_TILT_RAD);
      rollRad =
          MathUtil.clamp(
              rollRad + rollRateRadPerSec * SIM_DT_SECONDS, -MAX_ROBOT_TILT_RAD, MAX_ROBOT_TILT_RAD);
    }

    /**
     * Applies a small yaw response from tangential impact velocity so rotation comes from collisions.
     */
    private Pose2d applyCollisionYawResponse(Pose2d before, Pose2d corrected, ChassisSpeeds speeds) {
      Translation2d correction = corrected.getTranslation().minus(before.getTranslation());
      if (correction.getNorm() < 1e-8) {
        return corrected;
      }

      Translation2d normal = getCollisionNormal(correction);
      CollisionImpulse impulse = estimateCollisionImpulse(corrected, speeds, normal);
      if (impulse == null) {
        return corrected;
      }

      double yawImpulse = cross2d(impulse.contactOffset(), impulse.totalImpulse());
      double yawStep =
          MathUtil.clamp(
              (yawImpulse / getYawInertia()) * SIM_DT_SECONDS,
              -MAX_COLLISION_YAW_STEP_RAD,
              MAX_COLLISION_YAW_STEP_RAD);
      return new Pose2d(
          corrected.getTranslation(),
          corrected.getRotation().plus(new Rotation3d(0.0, 0.0, yawStep).toRotation2d()));
    }

    /**
     * Injects collision-induced pitch/roll rate so impacts visibly tilt the chassis.
     */
    private void applyCollisionTiltResponse(Pose2d before, Pose2d corrected, ChassisSpeeds speeds) {
      Translation2d correction = corrected.getTranslation().minus(before.getTranslation());
      if (correction.getNorm() < 1e-8) {
        return;
      }

      Translation2d normal = getCollisionNormal(correction);
      CollisionImpulse impulse = estimateCollisionImpulse(corrected, speeds, normal);
      if (impulse == null) {
        return;
      }

      double heading = corrected.getRotation().getRadians();
      double cos = Math.cos(heading);
      double sin = Math.sin(heading);
      Translation2d bodyImpulse =
          new Translation2d(
              impulse.totalImpulse().getX() * cos + impulse.totalImpulse().getY() * sin,
              -impulse.totalImpulse().getX() * sin + impulse.totalImpulse().getY() * cos);

      double leverArmZ = bumperHeightMeters * 0.5 - CENTER_OF_MASS_HEIGHT_METERS;
      double deltaPitchRate = (-leverArmZ * bodyImpulse.getX()) / getPitchRollInertia();
      double deltaRollRate = (leverArmZ * bodyImpulse.getY()) / getPitchRollInertia();

      pitchRateRadPerSec =
          MathUtil.clamp(
              pitchRateRadPerSec + deltaPitchRate,
              -MAX_COLLISION_TILT_RATE_RADPS,
              MAX_COLLISION_TILT_RATE_RADPS);
      rollRateRadPerSec =
          MathUtil.clamp(
              rollRateRadPerSec + deltaRollRate,
              -MAX_COLLISION_TILT_RATE_RADPS,
              MAX_COLLISION_TILT_RATE_RADPS);
    }

    private CollisionImpulse estimateCollisionImpulse(Pose2d pose, ChassisSpeeds speeds, Translation2d normal) {
      Translation2d tangent = new Translation2d(-normal.getY(), normal.getX());
      Translation2d contactOffset = getSupportPointOffset(pose, normal);
      double contactVelocityX = speeds.vxMetersPerSecond - (speeds.omegaRadiansPerSecond * contactOffset.getY());
      double contactVelocityY = speeds.vyMetersPerSecond + (speeds.omegaRadiansPerSecond * contactOffset.getX());
      Translation2d contactVelocity = new Translation2d(contactVelocityX, contactVelocityY);
      double normalSpeed = contactVelocity.dot(normal);
      if (normalSpeed >= -1e-4) {
        return null;
      }

      double normalDenominator =
          (1.0 / robotMassKg)
              + Math.pow(cross2d(contactOffset, normal), 2) / getYawInertia();
      double normalImpulseMagnitude =
          (-(1.0 + coefficientOfRestitution) * normalSpeed) / normalDenominator;
      if (normalImpulseMagnitude <= 0.0) {
        return null;
      }

      double tangentSpeed = contactVelocity.dot(tangent);
      double tangentDenominator =
          (1.0 / robotMassKg)
              + Math.pow(cross2d(contactOffset, tangent), 2) / getYawInertia();
      double rawFrictionImpulse = -tangentSpeed / tangentDenominator;
      double maxFrictionImpulse = COLLISION_FRICTION_COEFFICIENT * normalImpulseMagnitude;
      double tangentImpulseMagnitude =
          MathUtil.clamp(rawFrictionImpulse, -maxFrictionImpulse, maxFrictionImpulse);
      Translation2d totalImpulse =
          normal.times(normalImpulseMagnitude).plus(tangent.times(tangentImpulseMagnitude));
      return new CollisionImpulse(totalImpulse, contactOffset);
    }

    private Translation2d getSupportPointOffset(Pose2d pose, Translation2d outwardNormal) {
      Translation2d[] vertices = getRobotVertices(pose, robotLengthMeters * 0.5, robotWidthMeters * 0.5);
      Translation2d center = pose.getTranslation();
      Translation2d best = vertices[0];
      double bestDot = best.minus(center).dot(outwardNormal);
      for (int i = 1; i < vertices.length; i++) {
        Translation2d offset = vertices[i].minus(center);
        double dot = offset.dot(outwardNormal);
        if (dot > bestDot) {
          bestDot = dot;
          best = vertices[i];
        }
      }
      return best.minus(center);
    }

    private Translation2d getCollisionNormal(Translation2d correction) {
      if (lastCollisionNormal != null && lastCollisionNormal.getNorm() > 1e-8) {
        return lastCollisionNormal;
      }
      return correction.div(correction.getNorm()).times(-1.0);
    }

    private Translation2d[] getRobotVertices(Pose2d pose, double halfLength, double halfWidth) {
      double heading = pose.getRotation().getRadians();
      double cos = Math.cos(heading);
      double sin = Math.sin(heading);
      Translation2d center = pose.getTranslation();
      Translation2d forward = new Translation2d(cos * halfLength, sin * halfLength);
      Translation2d left = new Translation2d(-sin * halfWidth, cos * halfWidth);
      return new Translation2d[] {
        center.plus(forward).plus(left),
        center.plus(forward).minus(left),
        center.minus(forward).minus(left),
        center.minus(forward).plus(left)
      };
    }

    private Translation2d getEdgeNormal(Translation2d a, Translation2d b) {
      Translation2d edge = b.minus(a);
      double norm = edge.getNorm();
      if (norm < 1e-9) {
        return new Translation2d(1.0, 0.0);
      }
      return new Translation2d(-edge.getY() / norm, edge.getX() / norm);
    }

    private Projection projectOntoAxis(Translation2d[] points, Translation2d axis) {
      double min = points[0].dot(axis);
      double max = min;
      for (int i = 1; i < points.length; i++) {
        double value = points[i].dot(axis);
        if (value < min) {
          min = value;
        }
        if (value > max) {
          max = value;
        }
      }
      return new Projection(min, max);
    }

    private double cross2d(Translation2d a, Translation2d b) {
      return a.getX() * b.getY() - a.getY() * b.getX();
    }

    private double getYawInertia() {
      return (robotMassKg / 12.0)
          * ((robotLengthMeters * robotLengthMeters) + (robotWidthMeters * robotWidthMeters));
    }

    private double getPitchRollInertia() {
      double widthSquared = robotWidthMeters * robotWidthMeters;
      double heightSquared = ROBOT_BODY_HEIGHT_METERS * ROBOT_BODY_HEIGHT_METERS;
      return (robotMassKg / 12.0) * (widthSquared + heightSquared);
    }

    /**
     * Terrain profile used for driveline pitch/roll response and bump traversal.
     */
    private double getTerrainHeight(double xMeters, double yMeters) {
      if (yMeters < -0.2 || yMeters > FIELD_WIDTH_METERS + 0.2) {
        return 0.0;
      }
      boolean onLowLane = yMeters >= BUMP_LOW_Y_MIN && yMeters <= BUMP_LOW_Y_MAX;
      boolean onHighLane = yMeters >= BUMP_HIGH_Y_MIN && yMeters <= BUMP_HIGH_Y_MAX;
      if (!onLowLane && !onHighLane) {
        return 0.0;
      }

      // Match fuel's XZ bump geometry: blue bump and mirrored red bump.
      double blueBump = triangularBump(xMeters, BUMP_ENTRY_X, BUMP_PEAK_X, BUMP_EXIT_X, BUMP_HEIGHT);
      double redBump =
          triangularBump(
              xMeters,
              FIELD_LENGTH_METERS - BUMP_EXIT_X,
              FIELD_LENGTH_METERS - BUMP_PEAK_X,
              FIELD_LENGTH_METERS - BUMP_ENTRY_X,
              BUMP_HEIGHT);
      return Math.max(blueBump, redBump);
    }

    /**
     * Piecewise-linear triangular bump from x1 -> x2 -> x3.
     */
    private double triangularBump(double x, double x1, double x2, double x3, double peak) {
      if (x <= x1 || x >= x3) {
        return 0.0;
      }
      if (x < x2) {
        return peak * (x - x1) / (x2 - x1);
      }
      return peak * (x3 - x) / (x3 - x2);
    }

    /**
     * Pose3d composed from 2d odometry plus simulated terrain tilt.
     */
    private Pose3d getRobotPose3d(Pose2d pose) {
      return new Pose3d(
          pose.getX(),
          pose.getY(),
          chassisHeightMeters,
          new Rotation3d(rollRad, pitchRad, pose.getRotation().getRadians()));
    }

    /** Axis-aligned rectangle collider in field coordinates. */
    private static class ColliderRect {
      private final double xMin;
      private final double yMin;
      private final double xMax;
      private final double yMax;

      private ColliderRect(double xMin, double yMin, double xMax, double yMax) {
        this.xMin = xMin;
        this.yMin = yMin;
        this.xMax = xMax;
        this.yMax = yMax;
      }
    }

    /** Projection interval used in SAT overlap checks. */
    private record Projection(double min, double max) {}

    /** Collision solve output for one obstacle. */
    private record CollisionResult(Pose2d correctedPose, Translation2d collisionNormal, double overlapMeters) {}

    /** Impulse estimate for one collision contact. */
    private record CollisionImpulse(Translation2d totalImpulse, Translation2d contactOffset) {}
  }
}
