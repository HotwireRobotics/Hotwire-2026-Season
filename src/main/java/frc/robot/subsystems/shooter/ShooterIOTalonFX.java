package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Real hardware IO for shooter; uses TalonFX hardware. */
public class ShooterIOTalonFX implements ShooterIO {

  private static final double SLOT0_KV = 0.11451;
  private static final double SLOT0_KS = 0.19361;
  private static final double SLOT0_KP = 0.8;

  private final TalonFX leftShooter =
      new TalonFX(Constants.MotorIDs.s_shooterL, TunerConstants.kCANBus);
  private final TalonFX rightShooter =
      new TalonFX(Constants.MotorIDs.s_shooterR, TunerConstants.kCANBus);
  private final TalonFX feeder = new TalonFX(Constants.MotorIDs.s_feeder, TunerConstants.kCANBus);

  private final VoltageOut leftVoltageOutput = new VoltageOut(0.0);
  private final VoltageOut rightVoltageOutput = new VoltageOut(0.0);
  private final VoltageOut feederVoltageOutput = new VoltageOut(0.0);
  private final VelocityVoltage leftVelocityOutput = new VelocityVoltage(0.0);
  private final VelocityVoltage rightVelocityOutput = new VelocityVoltage(0.0);
  private final VelocityVoltage feederVelocityOutput = new VelocityVoltage(0.0);

  private final Slot0Configs shooterGains = new Slot0Configs();
  private final Slot0Configs feederGains = new Slot0Configs();

  /** Configures all shooter motors with default slot-0 gains. */
  public ShooterIOTalonFX() {
    shooterGains.withKV(SLOT0_KV);
    shooterGains.withKS(SLOT0_KS);
    shooterGains.withKP(SLOT0_KP);
    feederGains.withKV(SLOT0_KV);
    feederGains.withKS(SLOT0_KS);
    feederGains.withKP(SLOT0_KP);

    leftShooter.getConfigurator().apply(shooterGains);
    rightShooter.getConfigurator().apply(shooterGains);
    feeder.getConfigurator().apply(feederGains);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftShooterConnected = true;
    inputs.leftShooterPositionRad =
        Units.rotationsToRadians(leftShooter.getPosition().getValueAsDouble());
    inputs.leftShooterVelocityRadPerSec =
        Units.rotationsToRadians(leftShooter.getVelocity().getValueAsDouble());
    inputs.leftShooterAppliedVolts = leftShooter.getMotorVoltage().getValueAsDouble();
    inputs.leftShooterCurrentAmps = leftShooter.getSupplyCurrent().getValueAsDouble();
    inputs.leftShooterTempCelsius = leftShooter.getDeviceTemp().getValueAsDouble();

    inputs.rightShooterConnected = true;
    inputs.rightShooterPositionRad =
        Units.rotationsToRadians(rightShooter.getPosition().getValueAsDouble());
    inputs.rightShooterVelocityRadPerSec =
        Units.rotationsToRadians(rightShooter.getVelocity().getValueAsDouble());
    inputs.rightShooterAppliedVolts = rightShooter.getMotorVoltage().getValueAsDouble();
    inputs.rightShooterCurrentAmps = rightShooter.getSupplyCurrent().getValueAsDouble();
    inputs.rightShooterTempCelsius = rightShooter.getDeviceTemp().getValueAsDouble();

    inputs.feederConnected = true;
    inputs.feederPositionRad = Units.rotationsToRadians(feeder.getPosition().getValueAsDouble());
    inputs.feederVelocityRadPerSec =
        Units.rotationsToRadians(feeder.getVelocity().getValueAsDouble());
    inputs.feederAppliedVolts = feeder.getMotorVoltage().getValueAsDouble();
    inputs.feederCurrentAmps = feeder.getSupplyCurrent().getValueAsDouble();
    inputs.feederTempCelsius = feeder.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setLeftShooterOpenLoop(double output) {
    leftShooter.set(output);
  }

  @Override
  public void setRightShooterOpenLoop(double output) {
    rightShooter.set(output);
  }

  @Override
  public void setFeederOpenLoop(double output) {
    feeder.set(output);
  }

  @Override
  public void setLeftShooterVelocity(double velocityRadPerSec) {
    leftShooter.setControl(
        leftVelocityOutput.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setRightShooterVelocity(double velocityRadPerSec) {
    rightShooter.setControl(
        rightVelocityOutput.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setFeederVelocity(double velocityRadPerSec) {
    feeder.setControl(
        feederVelocityOutput.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setLeftShooterVoltage(double volts) {
    leftShooter.setControl(leftVoltageOutput.withOutput(volts));
  }

  @Override
  public void setRightShooterVoltage(double volts) {
    rightShooter.setControl(rightVoltageOutput.withOutput(volts));
  }

  @Override
  public void setFeederVoltage(double volts) {
    feeder.setControl(feederVoltageOutput.withOutput(volts));
  }

  @Override
  public void configureSlot0Kp(double kP) {
    shooterGains.withKP(kP);
    feederGains.withKP(kP);
    leftShooter.getConfigurator().apply(shooterGains);
    rightShooter.getConfigurator().apply(shooterGains);
    feeder.getConfigurator().apply(feederGains);
  }
}
