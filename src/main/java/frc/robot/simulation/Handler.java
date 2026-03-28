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
    private final Supplier<ChassisSpeeds> chassisSpeeds;

    private final Gamepiece gamepieceSimulation;
    
    public Handler(
        Supplier<AngularVelocity> velocity,
        BooleanSupplier shooter,
        BooleanSupplier intake,
        Supplier<Angle> wrist,
        Supplier<Pose2d> pose,
        Supplier<ChassisSpeeds> chassisSpeeds
    ) {
        this.velocity = velocity;

        doIntake = () -> {
            return intake.getAsBoolean() && (counter < limit) && (Math.random() > 0.99) && (pitch.lte(Degrees.of(3)));
        };

        target = wrist;

        doShoot = shooter;

        this.pose = pose;
        this.chassisSpeeds = chassisSpeeds;

        gamepieceSimulation = new Gamepiece();
        gamepieceSimulation.spawnStartingFuel();

        // Register a robot for collision with fuel
        gamepieceSimulation.registerRobot(
                Inches.of(35),
                Inches.of(35),
                Inches.of(4),
                this.pose, this.chassisSpeeds);

        gamepieceSimulation.registerIntake(
            Inches.of(17.5), Inches.of(24.118), Inches.of(-14.5), Inches.of(15.5), doIntake, this::intake);
        
        gamepieceSimulation.setSubticks(5);
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
        Logger.recordOutput("FinalComponentPoses", new Pose3d[] {
            new Pose3d(
                0.1958, 0.0, 0.21, 
                new Rotation3d(
                    Rotations.of(0), 
                    getWristPitch(), 
                    Rotations.of(0)
                ))
        });
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
}
