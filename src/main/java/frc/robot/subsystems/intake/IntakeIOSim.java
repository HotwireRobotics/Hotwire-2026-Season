package frc.robot.subsystems.intake;

/** Sim IO for intake; no physics model, inputs stay at zero for replay/sim without hardware. */
public class IntakeIOSim implements IntakeIO {
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollersConnected = true;
    inputs.rollersPositionRot = 0.0;
    inputs.rollersVelocityRpm = 0.0;
    inputs.rollersAppliedVolts = 0.0;
    inputs.rollersCurrentAmps = 0.0;
    inputs.rollersTempCelsius = 0.0;

    inputs.lowerConnected = true;
    inputs.lowerPositionRot = 0.0;
    inputs.lowerVelocityRpm = 0.0;
    inputs.lowerAppliedVolts = 0.0;
    inputs.lowerCurrentAmps = 0.0;
    inputs.lowerTempCelsius = 0.0;
  }
}
