package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import frc.robot.util.MotorTelemetry;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem with one feeder and left/right shooters. Supports percent output, velocity
 * (RPS), and voltage control. runShooter runs both wheels; runLeftShooter/runRightShooter run one
 * side for tuning or backup.
 */
public class ProtoShooter extends ModularSubsystem<ProtoShooter.Device> implements Systerface {
  private final SysIdRoutine m_sysIdRoutineRight;
  private final SysIdRoutine m_sysIdRoutineLeft;
  private final VoltageOut m_voltReq;
  private final VelocityVoltage m_velVolt;
  private final TalonFX m_feeder;
  private final TalonFX m_leftShooter;
  private final TalonFX m_rightShooter;
  private final Slot0Configs motorRPSControl;

  /** Logical devices for percent/velocity/voltage control and active-state tracking. */
  public enum Device {
    RIGHT_SHOOTER,
    LEFT_SHOOTER,
    BOTH_SHOOTER,
    FEEDER
  }

  public ProtoShooter() {
    m_feeder = new TalonFX(Constants.MotorIDs.s_feederR);
    m_leftShooter = new TalonFX(Constants.MotorIDs.s_shooterL);
    m_rightShooter = new TalonFX(Constants.MotorIDs.s_shooterR);

    motorRPSControl = new Slot0Configs();
    motorRPSControl.withKV(0.11451);
    motorRPSControl.withKS(0.19361);
    motorRPSControl.withKP(Constants.Shooter.kVelocityKp);

    final TalonFX[] bothShooters = {m_leftShooter, m_rightShooter};

    defineDevice(
        List.of(
            new DevicePointer(Device.RIGHT_SHOOTER, m_rightShooter),
            new DevicePointer(Device.LEFT_SHOOTER, m_leftShooter),
            new DevicePointer(Device.BOTH_SHOOTER, bothShooters),
            new DevicePointer(Device.FEEDER, m_feeder)));

    m_voltReq = new VoltageOut(0.0);
    m_velVolt = new VelocityVoltage(0.0);

    m_sysIdRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Shooter/SysIdState/Right", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.RIGHT_SHOOTER, voltage), null, this));
    m_sysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Shooter/SysIdState/Left", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.LEFT_SHOOTER, voltage), null, this));

    configureControl();
  }

  private enum State {
    STOPPED,
    SPINNING, // Running shooter
    FIRING // Running shooter & feeder
  }

  State state = State.STOPPED;

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/State", state.toString());

    Logger.recordOutput("Shooter/Active/BothShooter", isActiveDevice(Device.BOTH_SHOOTER));
    Logger.recordOutput("Shooter/Active/LeftShooter", isActiveDevice(Device.LEFT_SHOOTER));
    Logger.recordOutput("Shooter/Active/RightShooter", isActiveDevice(Device.RIGHT_SHOOTER));
    Logger.recordOutput("Shooter/Active/Feeder", isActiveDevice(Device.FEEDER));

    MotorTelemetry.logBasic("Shooter/Left", m_leftShooter);
    Logger.recordOutput(
        "Shooter/Left/VelocityRPS", m_leftShooter.getVelocity().getValueAsDouble(), "rps");
    Logger.recordOutput(
        "Shooter/Left/StatorCurrent", m_leftShooter.getStatorCurrent().getValueAsDouble(), "A");

    MotorTelemetry.logBasic("Shooter/Right", m_rightShooter);
    Logger.recordOutput(
        "Shooter/Right/VelocityRPS", m_rightShooter.getVelocity().getValueAsDouble(), "rps");
    Logger.recordOutput(
        "Shooter/Right/StatorCurrent", m_rightShooter.getStatorCurrent().getValueAsDouble(), "A");

    MotorTelemetry.logBasic("Shooter/Feeder", m_feeder);
    Logger.recordOutput(
        "Shooter/Feeder/VelocityRPS", m_feeder.getVelocity().getValueAsDouble(), "rps");
    Logger.recordOutput(
        "Shooter/Feeder/StatorCurrent", m_feeder.getStatorCurrent().getValueAsDouble(), "A");

    if (isActiveDevice(Device.BOTH_SHOOTER)) {
      state = isActiveDevice(Device.FEEDER) ? State.FIRING : State.SPINNING;
    } else {
      state = State.STOPPED;
    }
  }

  public Object getState() {
    return state;
  }

  // Device control methods
  public void runDevice(Device device, double speed) {
    for (TalonFX d : getDevices(device)) {
      d.set(speed);
    }

    if (speed == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }
  // Velocity control
  public void runDeviceVelocity(Device device, AngularVelocity velocity) {
    for (TalonFX d : getDevices(device)) {
      d.setControl(m_velVolt.withVelocity(velocity));
    }

    if (velocity.in(RotationsPerSecond) == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }
  // Voltage control
  public void runDeviceVoltage(Device device, Voltage voltage) {
    for (TalonFX d : getDevices(device)) {
      d.setControl(m_voltReq.withOutput(voltage.in(Volts)));
    }

    if (voltage.in(Volts) == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }

  // --- Main control: both shooters + feeder ---

  /** Runs both shooter wheels at the given percent output. */
  public void runShooter(double speed) {
    runDevice(Device.BOTH_SHOOTER, speed);
  }

  /** Runs the left shooter only (e.g. for tuning or single-side). */
  public void runLeftShooter(double speed) {
    runDevice(Device.LEFT_SHOOTER, speed);
  }

  /** Runs the right shooter only (e.g. for tuning or single-side). */
  public void runRightShooter(double speed) {
    runDevice(Device.RIGHT_SHOOTER, speed);
  }

  /** Runs the single feeder at the given percent output. */
  public void runFeeder(double speed) {
    runDevice(Device.FEEDER, speed);
  }

  /** Command: run both shooters and feeder at given percent. Main mechanism control. */
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

  /** Command: hold both shooters and feeder at fixed percentages while scheduled. */
  public Command holdMechanism(double feederSpeed, double shooterSpeed) {
    return Commands.startEnd(
        () -> {
          runShooter(shooterSpeed);
          runFeeder(feederSpeed);
        },
        () -> {
          runShooter(0);
          runFeeder(0);
        },
        this);
  }

  /** Command: hold both shooters and feeder at fixed velocity targets while scheduled. */
  public Command holdMechanismVelocity(AngularVelocity feederVel, AngularVelocity shooterVel) {
    return Commands.startEnd(
        () -> {
          runDeviceVelocity(Device.BOTH_SHOOTER, shooterVel);
          runDeviceVelocity(Device.FEEDER, feederVel);
        },
        () -> {
          runDeviceVelocity(Device.BOTH_SHOOTER, RotationsPerSecond.of(0));
          runDeviceVelocity(Device.FEEDER, RotationsPerSecond.of(0));
        },
        this);
  }

  /** Command: continuously refresh velocity setpoints from suppliers while scheduled. */
  public Command holdMechanismVelocity(
      Supplier<AngularVelocity> feederVel, Supplier<AngularVelocity> shooterVel) {
    return Commands.run(
            () -> {
              runDeviceVelocity(Device.BOTH_SHOOTER, shooterVel.get());
              runDeviceVelocity(Device.FEEDER, feederVel.get());
            },
            this)
        .finallyDo(
            () -> {
              runDeviceVelocity(Device.BOTH_SHOOTER, RotationsPerSecond.of(0));
              runDeviceVelocity(Device.FEEDER, RotationsPerSecond.of(0));
            });
  }

  public void configureProportional(double Kp) {
    motorRPSControl.withKP(Kp);
    configureControl();
  }

  private void configureControl() {
    m_rightShooter.getConfigurator().apply(motorRPSControl);
    m_leftShooter.getConfigurator().apply(motorRPSControl);
    m_feeder.getConfigurator().apply(motorRPSControl);
  }
  // Mechanism commands
  public Command sysIdQuasistaticRight(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.quasistatic(direction);
  }

  public Command sysIdDynamicRight(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.dynamic(direction);
  }

  public Command sysIdQuasistaticLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.quasistatic(direction);
  }

  public Command sysIdDynamicLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.dynamic(direction);
  }
}
