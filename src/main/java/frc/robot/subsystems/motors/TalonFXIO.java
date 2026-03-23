package frc.robot.subsystems.motors;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO implements MotorBase {

  // Declare device.
  private final TalonFX motor;

  // Control loops.
  private final VoltageOut voltageOut = new VoltageOut(0);
  private final PositionVoltage positionVoltage = new PositionVoltage(0);
  private final Follower follower = new Follower(0, MotorAlignmentValue.Aligned);

  // Configurator.
  private final TalonFXConfigurator configurator;

  public TalonFXIO(int deviceID) {
    motor = new TalonFX(deviceID);
    configurator = motor.getConfigurator();
  }

  public TalonFXIO(int deviceID, Current currentLimit) {
    this(deviceID);
    setCurrentLimit(currentLimit);
  }

  /** Set the motor output voltage. */
  @Override
  public void runVoltage(Voltage volts) {
    motor.setControl(voltageOut.withOutput(volts.in(Volts)));
  }

  /** Run to position. */
  @Override
  public void runPosition(Angle position) {
    motor.setControl(positionVoltage.withPosition(position));
  }

  /** Run to velocity. */
  @Override
  public void runVelocity(AngularVelocity velocity) {
    motor.setControl(positionVoltage.withVelocity(velocity));
  }

  /** Run at percent. */
  @Override
  public void runPercent(double percent) {
    motor.set(percent);
  }

  /** Stop the motor. */
  @Override
  public void stop() {
    motor.stopMotor();
  }

  /** Configure proportional gain (kP). */
  @Override
  public void configureProportional(double kP) {
    configurator.apply(new Slot0Configs().withKP(kP));
  }

  /** Configure integral gain (kI). */
  @Override
  public void configureIntegral(double kI) {
    configurator.apply(new Slot0Configs().withKI(kI));
  }

  /** Configure derivative gain (kD). */
  @Override
  public void configureDerivative(double kD) {
    configurator.apply(new Slot0Configs().withKD(kD));
  }

  /** Configure static friction feedforward (kS). */
  @Override
  public void configureStaticFriction(double kS) {
    configurator.apply(new Slot0Configs().withKS(kS));
  }

  /** Configure velocity feedforward (kV). */
  @Override
  public void configureVelocity(double kV) {
    configurator.apply(new Slot0Configs().withKV(kV));
  }

  /** Configure acceleration feedforward (kA). */
  @Override
  public void configureAcceleration(double kA) {
    configurator.apply(new Slot0Configs().withKA(kA));
  }

  /** Set the supply current limit. */
  @Override
  public void setCurrentLimit(Current limit) {
    CurrentLimitsConfigs current = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(limit.in(Amps));
    configurator.apply(current
        .withSupplyCurrentLimitEnable(true));
  }

  /** Set the motor direction. */
  @Override
  public void setDirection(Direction direction) {
    InvertedValue value = (direction == Direction.FORWARD) 
      ? InvertedValue.Clockwise_Positive 
      : InvertedValue.CounterClockwise_Positive;
    configurator.apply(
        new MotorOutputConfigs()
            .withInverted(value));
  }

  /** Set the neutral mode. */
  @Override
  public void setNeutralMode(NeutralMode mode) {
    NeutralModeValue neutral = (mode == NeutralMode.COAST) 
      ? NeutralModeValue.Coast 
      : NeutralModeValue.Brake;
    configurator.apply(
        new MotorOutputConfigs()
            .withNeutralMode(neutral));
  }

  /** Set the motor follower target. */
  @Override
  public void follow(int id, FollowerMode mode) {
    MotorAlignmentValue value = (mode == FollowerMode.ALIGNED) 
      ? MotorAlignmentValue.Aligned
      : MotorAlignmentValue.Opposed;
    motor.setControl(follower
      .withLeaderID(id)
      .withMotorAlignment(value));
  }

   /** Set the motor follower target from id. */
   @Override
   public void follow(Motor lead, FollowerMode mode) {
    follow(lead.getID(), mode);
   }

  /** Get measured velocity. */
  @Override
  public AngularVelocity getVelocity() {
    return motor.getVelocity().getValue();
  }

  /** Get measured position. */
  @Override
  public Angle getPosition() {
    return motor.getPosition().getValue();
  }

  /** Get measured current. */
  @Override
  public Current getCurrent() {
    return motor.getSupplyCurrent().getValue();
  }

  /** Get device id. */
  @Override
  public int getID() {
    return motor.getDeviceID();
  }

  /** Update all sensor and diagnostic inputs from the motor. */
  @Override
  public void updateInputs(MotorLogs inputs) {

    // Kinematics.
    /** Rotor position (rotations). */
    inputs.position = motor.getPosition().getValueAsDouble();

    /** Rotor velocity (rotations per second). */
    inputs.velocity = motor.getVelocity().getValueAsDouble();

    /** Rotor acceleration (rotations per second squared). */
    inputs.acceleration = motor.getAcceleration().getValueAsDouble();

    // Electrical.
    /** Voltage applied to the motor (V). */
    inputs.voltage = motor.getMotorVoltage().getValueAsDouble();

    /** Supply (bus) voltage (V). */
    inputs.supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();

    /** Supply current drawn from the bus (A). */
    inputs.current = motor.getSupplyCurrent().getValueAsDouble();

    /** Stator (phase) current inside the motor (A). */
    inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();

    // Thermal.
    /** Device temperature (°C). */
    inputs.temperature = motor.getDeviceTemp().getValueAsDouble();

    // Output and control.
    /** Applied output as a normalized value [-1, 1]. */
    inputs.appliedOutput = motor.getDutyCycle().getValueAsDouble();

    /** Duty cycle output (percentage of full output). */
    inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();

    // Diagnostics.
    /** True if the motor controller is connected and responding. */
    inputs.connected = motor.isConnected();
  }
}