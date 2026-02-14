package frc.robot.subsystems.shooter;

/** Sim IO for shooter; inputs stay at zero for replay/sim without hardware. */
public class ShooterIOSim implements ShooterIO {
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftShooterConnected = true;
    inputs.leftShooterPositionRot = 0.0;
    inputs.leftShooterVelocityRpm = 0.0;
    inputs.leftShooterAppliedVolts = 0.0;
    inputs.leftShooterCurrentAmps = 0.0;
    inputs.leftShooterTempCelsius = 0.0;

    inputs.rightShooterConnected = true;
    inputs.rightShooterPositionRot = 0.0;
    inputs.rightShooterVelocityRpm = 0.0;
    inputs.rightShooterAppliedVolts = 0.0;
    inputs.rightShooterCurrentAmps = 0.0;
    inputs.rightShooterTempCelsius = 0.0;

    inputs.feederConnected = true;
    inputs.feederPositionRot = 0.0;
    inputs.feederVelocityRpm = 0.0;
    inputs.feederAppliedVolts = 0.0;
    inputs.feederCurrentAmps = 0.0;
    inputs.feederTempCelsius = 0.0;
  }
}
