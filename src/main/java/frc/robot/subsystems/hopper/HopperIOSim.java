package frc.robot.subsystems.hopper;

/** Sim IO for hopper; inputs stay at zero for replay/sim without hardware. */
public class HopperIOSim implements HopperIO {
  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.upperConnected = true;
    inputs.upperPositionRot = 0.0;
    inputs.upperVelocityRpm = 0.0;
    inputs.upperAppliedVolts = 0.0;
    inputs.upperCurrentAmps = 0.0;
    inputs.upperTempCelsius = 0.0;

    inputs.lowerConnected = true;
    inputs.lowerPositionRot = 0.0;
    inputs.lowerVelocityRpm = 0.0;
    inputs.lowerAppliedVolts = 0.0;
    inputs.lowerCurrentAmps = 0.0;
    inputs.lowerTempCelsius = 0.0;
  }
}
