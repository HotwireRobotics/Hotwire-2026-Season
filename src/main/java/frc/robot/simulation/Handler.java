package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.Constants;

public class Handler {

    // Hopper count.
    private int counter = 0;
    // Declare supplier for shooting.
    private final Supplier<AngularVelocity> velocity;

    // Drive suppliers.
    private final Supplier<Pose2d> pose;
    private final Supplier<ChassisSpeeds> chassisSpeeds;

    private final Gamepiece gamepieceSimulation;
    
    public Handler(
        Supplier<AngularVelocity> velocity,
        Supplier<Pose2d> pose,
        Supplier<ChassisSpeeds> chassisSpeeds
    ) {
        this.velocity = velocity;

        this.pose = pose;
        this.chassisSpeeds = chassisSpeeds;

        gamepieceSimulation = new Gamepiece();
        gamepieceSimulation.clearFuel();

        // Register a robot for collision with fuel
        gamepieceSimulation.registerRobot(
                Inches.of(35),
                Inches.of(35),
                Inches.of(4),
                this.pose, this.chassisSpeeds);
        
        gamepieceSimulation.setSubticks(5);
        gamepieceSimulation.enableAirResistance();
        gamepieceSimulation.start();
    }

    /** Attempt to decrement the gamepiece counter. */
    public void shoot(
        AngularVelocity velocity
    ) {
        // Random chance of not firing based on the fact that we usually only shoot ~4 per second.
        Time time = Constants.Tempo.getTime();
        if (counter > 0 && time.in(Seconds) % 0.25 < 0.05) {
            gamepieceSimulation.launchFuel(lineate(velocity, Constants.Shooter.kAverageWheelRadius));
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
        gamepieceSimulation.updateSim();
    }

    private LinearVelocity lineate(AngularVelocity velocity, Distance radius) {
        return radius.times(Constants.Mathematics.TAU).per(Second).times(velocity.in(RotationsPerSecond));
    }
}
