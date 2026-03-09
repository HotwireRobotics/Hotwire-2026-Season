package frc.robot.subsystems.shooter;

/** Sim IO for shooter; inputs stay at zero for replay/sim without hardware. */
public class ShooterIOSim implements ShooterIO{
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update simulation state
        inputs.leftShooterConnected = true;
        inputs.rightShooterConnected = true;
        inputs.feederConnected = true;

        inputs.leftShooterPositionRad = 0.0;
        inputs.rightShooterPositionRad = 0.0;
        inputs.feederPositionRad = 0.0;
        inputs.leftShooterVelocityRadPerSec = 0.0;
        inputs.rightShooterVelocityRadPerSec = 0.0;
        inputs.feederVelocityRadPerSec = 0.0;
        inputs.leftShooterAppliedVolts = 0.0;
        inputs.rightShooterAppliedVolts = 0.0;
        inputs.feederAppliedVolts = 0.0;
        inputs.leftShooterCurrentAmps = 0.0;
        inputs.rightShooterCurrentAmps = 0.0;
        inputs.feederCurrentAmps = 0.0;
        inputs.leftShooterTemperatureC = 0.0;
        inputs.rightShooterTemperatureC = 0.0;
        inputs.feederTemperatureC = 0.0;
    }
}
