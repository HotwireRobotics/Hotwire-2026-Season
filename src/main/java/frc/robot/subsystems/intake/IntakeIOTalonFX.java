package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Real hardware IO implementation for intake (TalonFX rollers + lower). */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rollers = new TalonFX(Constants.MotorIDs.i_rollers, TunerConstants.kCANBus);
  private final TalonFX lower = new TalonFX(Constants.MotorIDs.i_follower, TunerConstants.kCANBus);

  public IntakeIOTalonFX() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollersConnected = rollers.getDeviceTemp().getStatus().isOK();
    inputs.rollersPositionRot = rollers.getPosition().getValueAsDouble();
    inputs.rollersVelocityRpm = rollers.getVelocity().getValueAsDouble() * 60.0;
    inputs.rollersAppliedVolts = rollers.getMotorVoltage().getValueAsDouble();
    inputs.rollersCurrentAmps = rollers.getSupplyCurrent().getValueAsDouble();
    inputs.rollersTempCelsius = rollers.getDeviceTemp().getValueAsDouble();

    inputs.lowerConnected = lower.getDeviceTemp().getStatus().isOK();
    inputs.lowerPositionRot = lower.getPosition().getValueAsDouble();
    inputs.lowerVelocityRpm = lower.getVelocity().getValueAsDouble() * 60.0;
    inputs.lowerAppliedVolts = lower.getMotorVoltage().getValueAsDouble();
    inputs.lowerCurrentAmps = lower.getSupplyCurrent().getValueAsDouble();
    inputs.lowerTempCelsius = lower.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setRollersOutput(double output) {
    rollers.set(output);
  }

  @Override
  public void setLowerOutput(double output) {
    lower.set(output);
  }
}
