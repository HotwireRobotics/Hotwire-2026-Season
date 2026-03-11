package frc.robot.subsystems.hopper;

/** Sim/replay-safe hopper IO with no hardware side effects. */
public class HopperIOSim implements HopperIO {
  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.upperConnected = true;
  }
}
