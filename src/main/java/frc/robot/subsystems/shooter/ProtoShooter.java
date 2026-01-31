package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoShooter extends SubsystemBase implements Systerface {
  private final TalonFX feeder;
  private final SysIdRoutine m_sysIdRoutineRight;
  private final SysIdRoutine m_SysIdRoutineLeft;
  private final VoltageOut m_voltReq;
  private final VelocityVoltage m_velVolt;
  private final ShooterModule rightModule;
  private final ShooterModule leftModule;

  public enum Device {
    FEEDER, RIGHT, LEFT, BOTH
  }

  private final HashMap<Device, Object> devices = new HashMap<Device, Object>();
  private List<Device> active = new ArrayList<Device>();

  private class ShooterModule {

    TalonFX shooter;
    TalonFX follower;

    public ShooterModule(int deviceID, int followerID) {
      shooter =  new TalonFX(deviceID);
      follower = new TalonFX(followerID);

      follower.setControl(
        new Follower(
          deviceID, MotorAlignmentValue.Opposed
        ));
    }

    public void runModule(double speed) {
      shooter.set(speed);
    }

    public void setControl(ControlRequest control) {
      shooter.setControl(control);
    }
  }

  public ProtoShooter() {
    feeder = new TalonFX(Constants.MotorIDs.s_feeder);

    rightModule = new ShooterModule(
      Constants.MotorIDs.s_shooterR,
      Constants.MotorIDs.s_followerR
    );

    leftModule = new ShooterModule(
      Constants.MotorIDs.s_shooterL,
      Constants.MotorIDs.s_followerL
    );

    TalonFX[] shooters = {leftModule.shooter, rightModule.shooter};

    devices.put(Device.FEEDER,feeder);
    devices.put(Device.RIGHT, rightModule.shooter);
    devices.put(Device.LEFT,  leftModule.shooter);
    devices.put(Device.BOTH,  shooters);


    m_voltReq = new VoltageOut(0.0);
    m_velVolt = new VelocityVoltage(0.0);

    m_sysIdRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.RIGHT, voltage),
                null,
                this));
    m_SysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.LEFT, voltage),
                null,
                this));
  }

  private enum State {
    STOPPED,
    HALTED,
    SPINNING, // Running shooter
    FIRING // Running shooter & feeder
  }

  State state = State.STOPPED;

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/State", state.toString());
    // Logger.recordOutput("Shooter/Feeder/Velocity", feeder.getVelocity().getValue());
    // Logger.recordOutput("Shooter/Shooter/Velocity", shooter.getVelocity().getValue());
    // Logger.recordOutput("Shooter/Follower/Velocity", follower.getVelocity().getValue());
    // Logger.recordOutput("Shooter/Feeder/Current", feeder.getSupplyCurrent().getValue());
    // Logger.recordOutput("Shooter/Shooter/Current", shooter.getSupplyCurrent().getValue());
    // Logger.recordOutput("Shooter/Follower/Current", follower.getSupplyCurrent().getValue());

    if (
      active.contains(Device.LEFT) ||
      active.contains(Device.BOTH) ||
      active.contains(Device.RIGHT)
    ) {
      if (active.contains(Device.FEEDER)) {
        state = State.FIRING;
      } else {
        state = State.SPINNING;
      }
    }
  }

  public Object getState() {
    return state;
  }

  public Command runMechanism(double speed) {
    return Commands.runOnce(
        () -> {
          rightModule.runModule(speed);
          leftModule .runModule(speed);
          feeder.set(0);
        });
  }

  private TalonFX[] getDevices(Device device) {
    Object group = devices.get(device);
    if (group instanceof TalonFX[]) {
      return ((TalonFX[]) group);
    } else {
      TalonFX[] items = {(TalonFX) group};
      return items;
    }
  }

  // Device control methods
  public void runDevice(Device device, double speed) {
    for (TalonFX d : getDevices(device)) {
      d.set(speed);
    }

    if (speed == 0) {
      active.remove(device);
    } else {
      active.add(device);
    }
  }

  public void runDeviceVelocity(Device device, AngularVelocity velocity) {
    for (TalonFX d : getDevices(device)) {
      d.setControl(m_velVolt.withVelocity(velocity));
    }

    if (velocity.in(RotationsPerSecond) == 0) {
      active.remove(device);
    } else {
      active.add(device);
    }
  }

  public void runDeviceVoltage(Device device, Voltage voltage) {
    for (TalonFX d : getDevices(device)) {
      d.setControl(m_voltReq.withOutput(voltage.in(Volts)));
    }

    if (voltage.in(Volts) == 0) {
      active.remove(device);
    } else {
      active.add(device);
    }
  }

  // Mechanism control commands
  public Command runMechanismVelocity(AngularVelocity feeder, AngularVelocity shooter) {
    return Commands.runOnce(
      () -> {
        runDeviceVelocity(Device.BOTH,  shooter);
        runDeviceVelocity(Device.FEEDER, feeder);
      });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.dynamic(direction);
  }
}
