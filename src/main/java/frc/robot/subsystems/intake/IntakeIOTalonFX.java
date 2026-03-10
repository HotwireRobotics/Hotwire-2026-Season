package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Real TalonFX-backed IO for intake rollers and arm. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rollers = new TalonFX(Constants.MotorIDs.i_rollers, TunerConstants.kCANBus);
  private final TalonFX arm = new TalonFX(Constants.MotorIDs.i_arm, TunerConstants.kCANBus);

  private final VoltageOut armVoltageRequest = new VoltageOut(0.0);
  private final PositionVoltage armPositionRequest = new PositionVoltage(Degrees.of(0));
  private final Slot0Configs armPositionGains = new Slot0Configs();

  /** Applies default position gains for arm control. */
  public IntakeIOTalonFX() {
    configureArmPositionKp(18.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollersConnected = true;
    inputs.rollersPositionRad = Units.rotationsToRadians(rollers.getPosition().getValueAsDouble());
    inputs.rollersVelocityRadPerSec =
        Units.rotationsToRadians(rollers.getVelocity().getValueAsDouble());
    inputs.rollersAppliedVolts = rollers.getMotorVoltage().getValueAsDouble();
    inputs.rollersCurrentAmps = rollers.getSupplyCurrent().getValueAsDouble();
    inputs.rollersTempCelsius = rollers.getDeviceTemp().getValueAsDouble();

    inputs.armConnected = true;
    inputs.armPositionRad = Units.rotationsToRadians(arm.getPosition().getValueAsDouble());
    inputs.armVelocityRadPerSec = Units.rotationsToRadians(arm.getVelocity().getValueAsDouble());
    inputs.armAppliedVolts = arm.getMotorVoltage().getValueAsDouble();
    inputs.armCurrentAmps = arm.getSupplyCurrent().getValueAsDouble();
    inputs.armTempCelsius = arm.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setRollersOpenLoop(double output) {
    rollers.set(output);
  }

  @Override
  public void setArmVoltage(double volts) {
    arm.setControl(armVoltageRequest.withOutput(volts));
  }

  @Override
  public void setArmPositionDegrees(double positionDegrees) {
    arm.setControl(armPositionRequest.withPosition(Degrees.of(positionDegrees)));
  }

  @Override
  public void configureArmPositionKp(double kP) {
    armPositionGains.withKP(kP);
    arm.getConfigurator().apply(armPositionGains);
  }
}
