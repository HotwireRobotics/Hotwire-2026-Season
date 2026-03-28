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
            double robotMassKg
        ) {
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
            Pose2d pose, ChassisSpeeds speeds, Consumer<Pose2d> setPose) {
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
                setPose.accept(correctedPose);
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
