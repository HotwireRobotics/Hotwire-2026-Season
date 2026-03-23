package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterReal implements ShooterBase {

    private final Supplier<AngularVelocity> velocity;
    
    public ShooterReal(Supplier<AngularVelocity> velocity) {
        this.velocity = velocity;
    }

    public AngularVelocity getTarget() {
        return velocity.get();
    }
}