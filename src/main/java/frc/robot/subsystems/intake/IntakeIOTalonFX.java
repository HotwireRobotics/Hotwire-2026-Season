package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rollers = new TalonFX(Constants.MotorIDs.i_rollers);

  private final StatusSignal<?> vel = rollers.getVelocity();
  private final StatusSignal<?> voltage = rollers.getMotorVoltage();
  private final StatusSignal<?> current = rollers.getSupplyCurrent();
  private final StatusSignal<?> temp = rollers.getDeviceTemp();

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(vel, voltage, current, temp);
    inputs.rollersConnected = status.isOK();
    // Velocity from Phoenix 6 is rot/s; log as rpm
    inputs.rollersVelocityRpm = vel.getValueAsDouble() * 60;
    inputs.rollersVoltage = voltage.getValueAsDouble();
    inputs.rollersCurrentAmps = current.getValueAsDouble();
    inputs.rollersTempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setRollersOutput(double output) {
    rollers.set(output);
  }
}
