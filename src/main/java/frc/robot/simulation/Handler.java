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
            ROBOT_LENGTH_WITH_BUMPERS,
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
        Logger.recordOutput("Components/Intake", new Pose3d[] {
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
    private static final double AIR_TILT_DAMPING = 0.65;
    private static final double SUPPORT_LAUNCH_VELOCITY_GAIN = 1.45;
    private static final double MAX_SUPPORT_LAUNCH_VELOCITY_MPS = 2.8;
    private static final double TILT_STIFFNESS = 42.0;
    private static final double TILT_DAMPING = 11.0;
    private static final double COLLISION_YAW_GAIN = 0.22;
    private static final double COLLISION_TILT_RATE_GAIN = 0.0208;
    private static final double HUB_COLLISION_TILT_MULTIPLIER = 2.5;
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
          FIELD_WIDTH_METERS - 1.57 + TRENCH_BLOCK_WIDTH)
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
              projectedHalfX - complianceX * 0.05,
              FIELD_LENGTH_METERS - projectedHalfX + complianceX * 0.05);
      double clampedY =
          MathUtil.clamp(
              pose.getY(),
              projectedHalfY - complianceY * 0.05,
              FIELD_WIDTH_METERS - projectedHalfY + complianceY * 0.05);

      boolean hitXWall = Math.abs(clampedX - pose.getX()) > 1e-6;
      boolean hitYWall = Math.abs(clampedY - pose.getY()) > 1e-6;
      Pose2d correctedPose = new Pose2d(clampedX, clampedY, pose.getRotation());
      correctedPose = resolveStaticColliders(correctedPose, halfLength, halfWidth, 8);

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
      for (int pass = 0; pass < passes; pass++) {
        boolean changed = false;
        for (ColliderRect rect : staticRectangles) {
          Pose2d before = corrected;
          corrected = resolveRectangleCollision(corrected, rect, halfLength, halfWidth);
          if (before.getTranslation().getDistance(corrected.getTranslation()) > 1e-8) {
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
    private Pose2d resolveRectangleCollision(
        Pose2d pose, ColliderRect rect, double halfLength, double halfWidth) {
      // Let robots traverse bump lanes without trench block sidewalls hard-locking movement.
    //   if (isTrenchBlockRect(rect) && isInBumpTraversalWindow(pose, halfLength, halfWidth)) {
    //     return pose;
    //   }

      double heading = pose.getRotation().getRadians();
      double projectedHalfX = Math.abs(Math.cos(heading)) * halfLength + Math.abs(Math.sin(heading)) * halfWidth;
      double projectedHalfY = Math.abs(Math.sin(heading)) * halfLength + Math.abs(Math.cos(heading)) * halfWidth;

      double left = pose.getX() - projectedHalfX;
      double right = pose.getX() + projectedHalfX;
      double bottom = pose.getY() - projectedHalfY;
      double top = pose.getY() + projectedHalfY;

      if (right <= rect.xMin || left >= rect.xMax || top <= rect.yMin || bottom >= rect.yMax) {
        return pose;
      }

      double pushLeft = rect.xMin - right;
      double pushRight = rect.xMax - left;
      double pushDown = rect.yMin - top;
      double pushUp = rect.yMax - bottom;

      Translation2d correction = new Translation2d(pushLeft, 0.0);
      if (Math.abs(pushRight) < Math.abs(correction.getX())) {
        correction = new Translation2d(pushRight, 0.0);
      }
      if (Math.abs(pushDown) < Math.abs(correction.getNorm())) {
        correction = new Translation2d(0.0, pushDown);
      }
      if (Math.abs(pushUp) < Math.abs(correction.getNorm())) {
        correction = new Translation2d(0.0, pushUp);
      }

      return new Pose2d(
          pose.getX() + correction.getX(),
          pose.getY() + correction.getY(),
          pose.getRotation());
    }

    private boolean isTrenchBlockRect(ColliderRect rect) {
      double eps = 1e-6;
      boolean blueBlock = Math.abs(rect.yMin - TRENCH_WIDTH) < eps;
      boolean redBlock = Math.abs(rect.yMin - (FIELD_WIDTH_METERS - 1.57)) < eps;
      return blueBlock || redBlock;
    }

    private boolean isInBumpTraversalWindow(Pose2d pose, double halfLength, double halfWidth) {
      double xMinBlue = BUMP_ENTRY_X - halfLength;
      double xMaxBlue = BUMP_EXIT_X + halfLength;
      double xMinRed = FIELD_LENGTH_METERS - BUMP_EXIT_X - halfLength;
      double xMaxRed = FIELD_LENGTH_METERS - BUMP_ENTRY_X + halfLength;
      boolean nearBlueBumpX = pose.getX() >= xMinBlue && pose.getX() <= xMaxBlue;
      boolean nearRedBumpX = pose.getX() >= xMinRed && pose.getX() <= xMaxRed;

      double yMinLow = BUMP_LOW_Y_MIN - halfWidth;
      double yMaxLow = BUMP_LOW_Y_MAX + halfWidth;
      double yMinHigh = BUMP_HIGH_Y_MIN - halfWidth;
      double yMaxHigh = BUMP_HIGH_Y_MAX + halfWidth;
      boolean onLowLane = pose.getY() >= yMinLow && pose.getY() <= yMaxLow;
      boolean onHighLane = pose.getY() >= yMinHigh && pose.getY() <= yMaxHigh;

      return (nearBlueBumpX || nearRedBumpX) && (onLowLane || onHighLane);
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
          MathUtil.clamp(terrainPitch + inertialPitch, -MAX_ROBOT_TILT_RAD, MAX_ROBOT_TILT_RAD);
      double targetRoll =
          MathUtil.clamp(terrainRoll + inertialRoll, -MAX_ROBOT_TILT_RAD, MAX_ROBOT_TILT_RAD);

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

      if (allWheelsGrounded && !chassisAirborne) {
        // Flat 4-wheel contact: rigid chassis should not wobble around.
        pitchRad = 0.0;
        rollRad = 0.0;
        pitchRateRadPerSec = 0.0;
        rollRateRadPerSec = 0.0;
        return;
      }

      // Rigid-body angular inertia over uneven terrain and in-air segments.
      double pitchAccel;
      double rollAccel;
      if (chassisAirborne) {
        pitchAccel = -AIR_TILT_DAMPING * pitchRateRadPerSec;
        rollAccel = -AIR_TILT_DAMPING * rollRateRadPerSec;
      } else {
        pitchAccel = TILT_STIFFNESS * (targetPitch - pitchRad) - TILT_DAMPING * pitchRateRadPerSec;
        rollAccel = TILT_STIFFNESS * (targetRoll - rollRad) - TILT_DAMPING * rollRateRadPerSec;
      }
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

      Translation2d normal = correction.div(correction.getNorm());
      Translation2d tangent = new Translation2d(-normal.getY(), normal.getX());
      Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      double normalSpeedIntoSurface = Math.max(0.0, -velocity.dot(normal));
      if (normalSpeedIntoSurface < 1e-3) {
        return corrected;
      }

      double tangentialSpeed = velocity.dot(tangent);
      double yawStep =
          MathUtil.clamp(
              tangentialSpeed * normalSpeedIntoSurface * COLLISION_YAW_GAIN,
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

      Translation2d normal = correction.div(correction.getNorm());
      Translation2d velocity = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      double normalSpeedIntoSurface = Math.max(0.0, -velocity.dot(normal));
      if (normalSpeedIntoSurface < 1e-3) {
        return;
      }

      double heading = corrected.getRotation().getRadians();
      double headingCos = Math.cos(heading);
      double headingSin = Math.sin(heading);
      double normalLongitudinal = normal.getX() * headingCos + normal.getY() * headingSin;
      double normalLateral = -normal.getX() * headingSin + normal.getY() * headingCos;

      double tiltRateImpulse = normalSpeedIntoSurface * COLLISION_TILT_RATE_GAIN;

      pitchRateRadPerSec =
          MathUtil.clamp(
              pitchRateRadPerSec - normalLongitudinal * tiltRateImpulse,
              -MAX_COLLISION_TILT_RATE_RADPS,
              MAX_COLLISION_TILT_RATE_RADPS);
      rollRateRadPerSec =
          MathUtil.clamp(
              rollRateRadPerSec + normalLateral * tiltRateImpulse,
              -MAX_COLLISION_TILT_RATE_RADPS,
              MAX_COLLISION_TILT_RATE_RADPS);
    }

    private boolean isNearHub(double xMeters, double yMeters) {
      double blueHubDx = xMeters - 4.61;
      double blueHubDy = yMeters - FIELD_WIDTH_METERS / 2.0;
      double redHubDx = xMeters - (FIELD_LENGTH_METERS - 4.61);
      double redHubDy = yMeters - FIELD_WIDTH_METERS / 2.0;
      double nearRadius = HUB_SIDE * 0.85;
      return Math.hypot(blueHubDx, blueHubDy) <= nearRadius || Math.hypot(redHubDx, redHubDy) <= nearRadius;
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
  }
}
