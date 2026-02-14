package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
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
import java.util.function.Supplier;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem with left/right modules (shooter + feeder per side). Supports percent output,
 * velocity (RPS), and voltage control, plus SysId routines per side.
 */
public class ProtoShooter extends ModularSubsystem implements Systerface {
  private final SysIdRoutine m_sysIdRoutineRight;
  private final SysIdRoutine m_sysIdRoutineLeft;
  private final VoltageOut m_voltReq;
  private final VelocityVoltage m_velVolt;
  private final ShooterModule rightModule;
  private final ShooterModule leftModule;
  private final Slot0Configs motorRPSControl;

  public enum Device {
    RIGHT_FEEDER,
    RIGHT_SHOOTER,
    LEFT_FEEDER,
    LEFT_SHOOTER,
    BOTH_FEEDER,
    BOTH_SHOOTER
  }

  // private final HashMap<Device, Object> devices = new HashMap<Device, Object>();
  // private List<Device> active = new ArrayList<Device>();

  private class ShooterModule {

    TalonFX shooter;
    TalonFX feeder;

    public ShooterModule(@NotNull int deviceID, @NotNull int followerID) {
      shooter = new TalonFX(deviceID);
      feeder = new TalonFX(followerID);
    }

    public void runModule(@Nullable double speed) {
      shooter.set(speed);
      feeder.set(speed);
    }

    public void setControl(@Nullable ControlRequest control) {
      shooter.setControl(control);
    }
  }

  public ProtoShooter() {

    rightModule = new ShooterModule(Constants.MotorIDs.s_shooterR, Constants.MotorIDs.s_feederR);
    leftModule = new ShooterModule(Constants.MotorIDs.s_shooterL, Constants.MotorIDs.s_feederR);

    motorRPSControl = new Slot0Configs();
    motorRPSControl.withKV(0.11451);
    motorRPSControl.withKS(0.19361);
    motorRPSControl.withKP(0.8);

    final TalonFX[] shooters = {leftModule.shooter, rightModule.shooter};
    final TalonFX[] feeders = {leftModule.feeder, rightModule.feeder};

    defineDevice(
        new DevicePointer(Device.RIGHT_FEEDER, rightModule.feeder),
        new DevicePointer(Device.LEFT_FEEDER, rightModule.feeder),
        new DevicePointer(Device.RIGHT_SHOOTER, rightModule.shooter),
        new DevicePointer(Device.LEFT_SHOOTER, leftModule.shooter),
        new DevicePointer(Device.BOTH_FEEDER, feeders),
        new DevicePointer(Device.BOTH_SHOOTER, shooters));

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

    // for (TalonFX motor : getDevices(shooters)) {
    //   motor.getConfigurator().apply(shooterRPSControl);
    // }
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

    // Log position (rot), velocity (rpm), voltage, current, temp with unit metadata
    Logger.recordOutput(
        "Shooter/Left/Position", leftModule.shooter.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Shooter/Left/Velocity", leftModule.shooter.getVelocity().getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Shooter/Left/Voltage", leftModule.shooter.getMotorVoltage().getValueAsDouble(), "V");
    Logger.recordOutput(
        "Shooter/Left/Current", leftModule.shooter.getSupplyCurrent().getValueAsDouble(), "A");
    Logger.recordOutput(
        "Shooter/Left/Temperature", leftModule.shooter.getDeviceTemp().getValueAsDouble(), "°C");
    Logger.recordOutput(
        "Shooter/Right/Position", rightModule.shooter.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Shooter/Right/Velocity", rightModule.shooter.getVelocity().getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Shooter/Right/Voltage", rightModule.shooter.getMotorVoltage().getValueAsDouble(), "V");
    Logger.recordOutput(
        "Shooter/Right/Current", rightModule.shooter.getSupplyCurrent().getValueAsDouble(), "A");
    Logger.recordOutput(
        "Shooter/Right/Temperature", rightModule.shooter.getDeviceTemp().getValueAsDouble(), "°C");
    Logger.recordOutput(
        "Shooter/LeftFollower/Position", leftModule.feeder.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Shooter/LeftFollower/Velocity",
        leftModule.feeder.getVelocity().getValueAsDouble() * 60,
        "rpm");
    Logger.recordOutput(
        "Shooter/LeftFollower/Voltage",
        leftModule.feeder.getMotorVoltage().getValueAsDouble(),
        "V");
    Logger.recordOutput(
        "Shooter/LeftFollower/Current",
        leftModule.feeder.getSupplyCurrent().getValueAsDouble(),
        "A");
    Logger.recordOutput(
        "Shooter/LeftFollower/Temperature",
        leftModule.feeder.getDeviceTemp().getValueAsDouble(),
        "°C");
    Logger.recordOutput(
        "Shooter/RightFollower/Position",
        rightModule.feeder.getPosition().getValueAsDouble(),
        "rot");
    Logger.recordOutput(
        "Shooter/RightFollower/Velocity",
        rightModule.feeder.getVelocity().getValueAsDouble() * 60,
        "rpm");
    Logger.recordOutput(
        "Shooter/RightFollower/Voltage",
        rightModule.feeder.getMotorVoltage().getValueAsDouble(),
        "V");
    Logger.recordOutput(
        "Shooter/RightFollower/Current",
        rightModule.feeder.getSupplyCurrent().getValueAsDouble(),
        "A");
    Logger.recordOutput(
        "Shooter/RightFollower/Temperature",
        rightModule.feeder.getDeviceTemp().getValueAsDouble(),
        "°C");

    if (isActiveDevice(Device.BOTH_SHOOTER)) {
      if (isActiveDevice(Device.BOTH_FEEDER)) {
        state = State.FIRING;
      } else {
        state = State.SPINNING;
      }
    } else {
      state = State.STOPPED;
    }
  }

  public Object getState() {
    return state;
  }

  // Device control methods
  public void runDevice(@NotNull Device device, @NotNull double speed) {
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
  public void runDeviceVelocity(@NotNull Device device, @NotNull AngularVelocity velocity) {
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
  public void runDeviceVoltage(@NotNull Device device, @NotNull Voltage voltage) {
    for (TalonFX d : getDevices(device)) {
      d.setControl(m_voltReq.withOutput(voltage.in(Volts)));
    }

    if (voltage.in(Volts) == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }

  // Mechanism control commands
  public Command runMechanism(@NotNull double feeder, @NotNull double shooter) {
    return Commands.runOnce(
        () -> {
          runDevice(Device.BOTH_SHOOTER, shooter);
          runDevice(Device.BOTH_FEEDER, feeder);
        });
  }

  public Command runMechanismVelocity(
      @NotNull AngularVelocity feeder, @NotNull AngularVelocity shooter) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.BOTH_SHOOTER, shooter);
          runDeviceVelocity(Device.BOTH_FEEDER, feeder);
        });
  }

  public Command runMechanismVelocity(
      Supplier<AngularVelocity> feeder, Supplier<AngularVelocity> shooter) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.BOTH_SHOOTER, shooter.get());
          runDeviceVelocity(Device.BOTH_FEEDER, feeder.get());
        });
  }

  public Command runRightModule(@NotNull double feeder, @NotNull double shooter) {
    return Commands.runOnce(
        () -> {
          runDevice(Device.RIGHT_SHOOTER, shooter);
          runDevice(Device.RIGHT_FEEDER, feeder);
        });
  }

  public Command runLeftModule(@NotNull double feeder, @NotNull double shooter) {
    return Commands.runOnce(
        () -> {
          runDevice(Device.RIGHT_SHOOTER, shooter);
          runDevice(Device.RIGHT_FEEDER, feeder);
        });
  }

  public Command runRightModuleVelocity(
      Supplier<AngularVelocity> feeder, Supplier<AngularVelocity> shooter) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.RIGHT_SHOOTER, shooter.get());
          runDeviceVelocity(Device.RIGHT_FEEDER, feeder.get());
        });
  }

  public Command runLeftModuleVelocity(
      Supplier<AngularVelocity> feeder, Supplier<AngularVelocity> shooter) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.LEFT_SHOOTER, shooter.get());
          runDeviceVelocity(Device.LEFT_FEEDER, feeder.get());
        });
  }

  public void configureProportional(@Nullable double Kp) {
    motorRPSControl.withKP(Kp);
    configureControl();
  }

  private void configureControl() {
    rightModule.shooter.getConfigurator().apply(motorRPSControl);
    leftModule.shooter.getConfigurator().apply(motorRPSControl);
    rightModule.feeder.getConfigurator().apply(motorRPSControl);
    leftModule.feeder.getConfigurator().apply(motorRPSControl);
  }
  // Mechanism commands
  public Command sysIdQuasistaticRight(@NotNull SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.quasistatic(direction);
  }

  public Command sysIdDynamicRight(@NotNull SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.dynamic(direction);
  }

  public Command sysIdQuasistaticLeft(@NotNull SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.quasistatic(direction);
  }

  public Command sysIdDynamicLeft(@NotNull SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.dynamic(direction);
  }
}
