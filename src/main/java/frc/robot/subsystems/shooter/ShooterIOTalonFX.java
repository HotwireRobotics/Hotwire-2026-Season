package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Real hardware IO implementation for shooter (left/right shooter + feeder TalonFX). */
public class ShooterIOTalonFX implements ShooterIO {

  private static final double SLOT0_KV = 0.11451;
  private static final double SLOT0_KS = 0.19361;
  private static final double SLOT0_KP = 0.8;

  private final TalonFX leftShooter =
      new TalonFX(Constants.MotorIDs.s_shooterL, TunerConstants.kCANBus);
  private final TalonFX rightShooter =
      new TalonFX(Constants.MotorIDs.s_shooterR, TunerConstants.kCANBus);
  private final TalonFX feeder = new TalonFX(Constants.MotorIDs.s_feederR, TunerConstants.kCANBus);

  private final VoltageOut leftVoltageOut = new VoltageOut(0.0);
  private final VoltageOut rightVoltageOut = new VoltageOut(0.0);
  private final VoltageOut feederVoltageOut = new VoltageOut(0.0);
  private final VelocityVoltage leftVelocityVoltage = new VelocityVoltage(0.0);
  private final VelocityVoltage rightVelocityVoltage = new VelocityVoltage(0.0);
  private final VelocityVoltage feederVelocityVoltage = new VelocityVoltage(0.0);

  private final Slot0Configs slot0Config = new Slot0Configs();

  public ShooterIOTalonFX() {
    slot0Config.withKV(SLOT0_KV).withKS(SLOT0_KS).withKP(SLOT0_KP);
    applySlot0();
  }

  private void applySlot0() {
    leftShooter.getConfigurator().apply(slot0Config);
    rightShooter.getConfigurator().apply(slot0Config);
    feeder.getConfigurator().apply(slot0Config);
  }

  @Override
  public void configureSlot0Kp(double kp) {
    slot0Config.withKP(kp);
    applySlot0();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftShooterConnected = leftShooter.getDeviceTemp().getStatus().isOK();
    inputs.leftShooterPositionRot = leftShooter.getPosition().getValueAsDouble();
    inputs.leftShooterVelocityRpm = leftShooter.getVelocity().getValueAsDouble() * 60.0;
    inputs.leftShooterAppliedVolts = leftShooter.getMotorVoltage().getValueAsDouble();
    inputs.leftShooterCurrentAmps = leftShooter.getSupplyCurrent().getValueAsDouble();
    inputs.leftShooterTempCelsius = leftShooter.getDeviceTemp().getValueAsDouble();

    inputs.rightShooterConnected = rightShooter.getDeviceTemp().getStatus().isOK();
    inputs.rightShooterPositionRot = rightShooter.getPosition().getValueAsDouble();
    inputs.rightShooterVelocityRpm = rightShooter.getVelocity().getValueAsDouble() * 60.0;
    inputs.rightShooterAppliedVolts = rightShooter.getMotorVoltage().getValueAsDouble();
    inputs.rightShooterCurrentAmps = rightShooter.getSupplyCurrent().getValueAsDouble();
    inputs.rightShooterTempCelsius = rightShooter.getDeviceTemp().getValueAsDouble();

    inputs.feederConnected = feeder.getDeviceTemp().getStatus().isOK();
    inputs.feederPositionRot = feeder.getPosition().getValueAsDouble();
    inputs.feederVelocityRpm = feeder.getVelocity().getValueAsDouble() * 60.0;
    inputs.feederAppliedVolts = feeder.getMotorVoltage().getValueAsDouble();
    inputs.feederCurrentAmps = feeder.getSupplyCurrent().getValueAsDouble();
    inputs.feederTempCelsius = feeder.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setLeftShooterPercent(double output) {
    leftShooter.set(output);
  }

  @Override
  public void setRightShooterPercent(double output) {
    rightShooter.set(output);
  }

  @Override
  public void setFeederPercent(double output) {
    feeder.set(output);
  }

  @Override
  public void setLeftShooterVelocityRotPerSec(double rps) {
    leftShooter.setControl(leftVelocityVoltage.withVelocity(RotationsPerSecond.of(rps)));
  }

  @Override
  public void setRightShooterVelocityRotPerSec(double rps) {
    rightShooter.setControl(rightVelocityVoltage.withVelocity(RotationsPerSecond.of(rps)));
  }

  @Override
  public void setFeederVelocityRotPerSec(double rps) {
    feeder.setControl(feederVelocityVoltage.withVelocity(RotationsPerSecond.of(rps)));
  }

  @Override
  public void setLeftShooterVoltage(double volts) {
    leftShooter.setControl(leftVoltageOut.withOutput(volts));
  }

  @Override
  public void setRightShooterVoltage(double volts) {
    rightShooter.setControl(rightVoltageOut.withOutput(volts));
  }

  @Override
  public void setFeederVoltage(double volts) {
    feeder.setControl(feederVoltageOut.withOutput(volts));
  }
}
