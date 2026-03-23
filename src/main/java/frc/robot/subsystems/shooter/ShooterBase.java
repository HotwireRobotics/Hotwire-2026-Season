package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

public interface ShooterBase {

    AngularVelocity getTarget();

    Command run();
    Command halt();
}
