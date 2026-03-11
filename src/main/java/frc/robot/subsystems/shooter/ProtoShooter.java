package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Systerface;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem with one feeder and left/right shooters. Supports percent output, velocity
 * (RPS), and voltage control. runShooter runs both wheels; runLeftShooter/runRightShooter run one
 * side for tuning or backup.
 */
public class ProtoShooter extends edu.wpi.first.wpilibj2.command.SubsystemBase implements Systerface {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final EnumSet<Device> activeDevices = EnumSet.noneOf(Device.class);
  private final SysIdRoutine m_sysIdRoutineRight;
  private final SysIdRoutine m_sysIdRoutineLeft;

  /** Logical devices for percent/velocity/voltage control and active-state tracking. */
  public enum Device {
    RIGHT_SHOOTER,
    LEFT_SHOOTER,
    BOTH_SHOOTER,
    FEEDER
  }

  /** Constructs shooter with the selected IO implementation. */
  public ProtoShooter(ShooterIO io) {
    this.io = io;

    m_sysIdRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState/Right", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.RIGHT_SHOOTER, voltage), null, this));
    m_sysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState/Left", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.LEFT_SHOOTER, voltage), null, this));
  }

  private enum State {
    STOPPED,
    SPINNING,
    FIRING
  }

  private State state = State.STOPPED;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/Active/BothShooter", activeDevices.contains(Device.BOTH_SHOOTER));
    Logger.recordOutput("Shooter/Active/LeftShooter", activeDevices.contains(Device.LEFT_SHOOTER));
    Logger.recordOutput(
        "Shooter/Active/RightShooter", activeDevices.contains(Device.RIGHT_SHOOTER));
    Logger.recordOutput("Shooter/Active/Feeder", activeDevices.contains(Device.FEEDER));

    if (activeDevices.contains(Device.BOTH_SHOOTER)) {
      state = activeDevices.contains(Device.FEEDER) ? State.FIRING : State.SPINNING;
    } else {
      state = State.STOPPED;
    }
    Logger.recordOutput("Shooter/State", state.toString());
  }

  @Override
  public Object getState() {
    return state;
  }

  /** Runs a selected logical device open-loop. */
  public void runDevice(Device device, double speed) {
    switch (device) {
      case RIGHT_SHOOTER -> io.setRightShooterOpenLoop(speed);
      case LEFT_SHOOTER -> io.setLeftShooterOpenLoop(speed);
      case BOTH_SHOOTER -> {
        io.setLeftShooterOpenLoop(speed);
        io.setRightShooterOpenLoop(speed);
      }
      case FEEDER -> io.setFeederOpenLoop(speed);
    }
    setDeviceActivity(device, speed != 0.0);
  }

  /** Runs a selected logical device at velocity. */
  public void runDeviceVelocity(Device device, AngularVelocity velocity) {
    final double velocityRadPerSec = Units.rotationsToRadians(velocity.in(RotationsPerSecond));
    switch (device) {
      case RIGHT_SHOOTER -> io.setRightShooterVelocity(velocityRadPerSec);
      case LEFT_SHOOTER -> io.setLeftShooterVelocity(velocityRadPerSec);
      case BOTH_SHOOTER -> {
        io.setLeftShooterVelocity(velocityRadPerSec);
        io.setRightShooterVelocity(velocityRadPerSec);
      }
      case FEEDER -> io.setFeederVelocity(velocityRadPerSec);
    }
    setDeviceActivity(device, velocity.in(RotationsPerSecond) != 0.0);
  }

  /** Runs a selected logical device at a voltage setpoint. */
  public void runDeviceVoltage(Device device, Voltage voltage) {
    final double volts = voltage.in(Volts);
    switch (device) {
      case RIGHT_SHOOTER -> io.setRightShooterVoltage(volts);
      case LEFT_SHOOTER -> io.setLeftShooterVoltage(volts);
      case BOTH_SHOOTER -> {
        io.setLeftShooterVoltage(volts);
        io.setRightShooterVoltage(volts);
      }
      case FEEDER -> io.setFeederVoltage(volts);
    }
    setDeviceActivity(device, volts != 0.0);
  }

  /** Tracks active logical devices for state and replay logs. */
  private void setDeviceActivity(Device device, boolean active) {
    if (device == Device.BOTH_SHOOTER) {
      setSingleDevice(Device.LEFT_SHOOTER, active);
      setSingleDevice(Device.RIGHT_SHOOTER, active);
      setSingleDevice(Device.BOTH_SHOOTER, active);
      return;
    }
    setSingleDevice(device, active);
    if (!active
        && !activeDevices.contains(Device.LEFT_SHOOTER)
        && !activeDevices.contains(Device.RIGHT_SHOOTER)) {
      activeDevices.remove(Device.BOTH_SHOOTER);
    }
    if (active && (device == Device.LEFT_SHOOTER || device == Device.RIGHT_SHOOTER)) {
      activeDevices.add(Device.BOTH_SHOOTER);
    }
  }

  /** Adds or removes one device from the active set. */
  private void setSingleDevice(Device device, boolean active) {
    if (active) {
      activeDevices.add(device);
    } else {
      activeDevices.remove(device);
    }
  }

  /** Runs both shooter wheels at the given percent output. */
  public void runShooter(double speed) {
    runDevice(Device.BOTH_SHOOTER, speed);
  }

  /** Runs the left shooter only. */
  public void runLeftShooter(double speed) {
    runDevice(Device.LEFT_SHOOTER, speed);
  }

  /** Runs the right shooter only. */
  public void runRightShooter(double speed) {
    runDevice(Device.RIGHT_SHOOTER, speed);
  }

  /** Runs the feeder at the given percent output. */
  public void runFeeder(double speed) {
    runDevice(Device.FEEDER, speed);
  }

  /** Command: run both shooters and feeder at given percent. */
  public Command runMechanism(double feederSpeed, double shooterSpeed) {
    return Commands.runOnce(
        () -> {
          runShooter(shooterSpeed);
          runFeeder(feederSpeed);
        });
  }

  /** Command: run both shooters and feeder at given velocities. */
  public Command runMechanismVelocity(AngularVelocity feederVel, AngularVelocity shooterVel) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.BOTH_SHOOTER, shooterVel);
          runDeviceVelocity(Device.FEEDER, feederVel);
        });
  }

  /** Command: run both shooters and feeder at supplier-derived velocities. */
  public Command runMechanismVelocity(
      Supplier<AngularVelocity> feederVel, Supplier<AngularVelocity> shooterVel) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.BOTH_SHOOTER, shooterVel.get());
          runDeviceVelocity(Device.FEEDER, feederVel.get());
        });
  }

  /** Updates shooter proportional gains in IO implementation. */
  public void configureProportional(double kP) {
    io.configureSlot0Kp(kP);
  }

  /** SysId command for right shooter quasistatic tests. */
  public Command sysIdQuasistaticRight(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.quasistatic(direction);
  }

  /** SysId command for right shooter dynamic tests. */
  public Command sysIdDynamicRight(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.dynamic(direction);
  }

  /** SysId command for left shooter quasistatic tests. */
  public Command sysIdQuasistaticLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.quasistatic(direction);
  }

  /** SysId command for left shooter dynamic tests. */
  public Command sysIdDynamicLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.dynamic(direction);
  }
}
