package frc.robot.subsystems.shooter;

/** Sim IO for shooter; inputs stay at zero for replay/sim without hardware. */
public class ShooterIOSim implements ShooterIO {
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftShooterConnected = true;
    inputs.rightShooterConnected = true;
    inputs.feederConnected = true;
  }
}
