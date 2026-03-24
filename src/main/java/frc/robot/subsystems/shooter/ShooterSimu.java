package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterSimu {
    private final Supplier<AngularVelocity> velocity;
    
    public ShooterSimu(Supplier<AngularVelocity> velocity) {
        this.velocity = velocity;
    }

    public AngularVelocity getTarget() {
        return velocity.get();
    }

    // public Command run
}
