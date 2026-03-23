package frc.robot.simulation;

import java.util.function.Consumer;

import edu.wpi.first.units.measure.AngularVelocity;

public class Handler {

    private int counter = 0;
    
    public Handler(
        Consumer<AngularVelocity> shoot
    ) {
        
    }

    public void shoot(
        AngularVelocity velocity
    ) {
        
    }

    /** Increment gamepiece counter. */
    public void intake() {
        this.counter ++;
    }

    /** Initialize with gamepiece(s). */
    public void setCounter(
        int count
    ) {
        counter = count;
    }
}
