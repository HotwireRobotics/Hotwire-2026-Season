package frc.robot.subsystems.intake;

/** Sim/replay-safe intake IO with no hardware side effects. */
public class IntakeIOSim implements IntakeIO {
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollersConnected = true;
    inputs.armConnected = true;
  }
}
