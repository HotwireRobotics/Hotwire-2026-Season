package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import frc.robot.util.MotorTelemetry;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ProtoIntake extends ModularSubsystem<ProtoIntake.Device> implements Systerface {
  private final TalonFX rollers;
  private final TalonFX lower;
  // private final TalonFX arm;

  public enum Device {
    ROLLERS,
    LOWER,
    ARM
  }

  // private final SysIdRoutine m_sysIdRoutineRight;
  // private final SysIdRoutine m_sysIdRoutineLeft;
  // private final SysIdRoutine m_sysIdRoutineARM;

  public ProtoIntake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    lower = new TalonFX(Constants.MotorIDs.i_follower);
    defineDevice(
        List.of(
            new DevicePointer(Device.ROLLERS, rollers), new DevicePointer(Device.LOWER, lower)));
  }

  private enum State {
    STOPPED,
    INTAKING // Running rollers
  }

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/State", state.toString());

    /**
     * Logs position of (rot)
     *
     * <p>Logs velocity in (rpm)
     *
     * <p>Logs voltage current
     *
     * <p>Logs temp with unit metadata
     */
    MotorTelemetry.logBasic("Intake/Rollers", rollers);

    if (isActiveDevice(Device.ROLLERS)) {
      state = State.INTAKING;
    } else {
      state = State.STOPPED;
    }
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
  /**
   * Allows you to limit the voltage of the intake rollers
   *
   * @param device
   * @param volts
   */
  public void runDeviceVoltage(Device device, double volts) {
    for (TalonFX d : getDevices(device)) {
      d.setVoltage(volts);
    }

    if (volts == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }
  /**
   * Allows you to limit the speed of the intake rollers
   *
   * @param speed
   * @return
   */
  public Command runIntake(double speed) {
    return Commands.runOnce(
        () -> {
          runDevice(Device.ROLLERS, speed);
          runDevice(Device.LOWER, speed);
        });
  }

  /** Holds both intake motors at a shared speed while scheduled, then stops on release. */
  public Command holdIntake(double speed) {
    return Commands.startEnd(
        () -> {
          runDevice(Device.ROLLERS, speed);
          runDevice(Device.LOWER, speed);
        },
        () -> {
          runDevice(Device.ROLLERS, 0.0);
          runDevice(Device.LOWER, 0.0);
        },
        this);
  }

  public Command moveIntake(double speed) {
    return Commands.run(
        () -> {
          runDevice(Device.ARM, speed);
        });
  }
  /**
   * Commands mentioned above for m_sysIdRoutineRight and m_sysIdRoutineLeft
   *
   * @param direction
   * @return m_sysIdRoutineRight, m_sysIdRoutineLeft
   */
  // public Command sysIdQuasistaticRight(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutineRight.quasistatic(direction);
  // }

  // public Command sysIdDynamicRight(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutineRight.dynamic(direction);
  // }

  // public Command sysIdQuasistaticLeft(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutineLeft.quasistatic(direction);
  // }

  // public Command sysIdDynamicLeft(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutineLeft.dynamic(direction);
  // }

  // public Command sysIdQuasistaticARM(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutineARM.quasistatic(direction);
  // }

  // public Command sysIdDynamicARM(SysIdRoutine.Direction direction) {
  //   return m_sysIdRoutineARM.dynamic(direction);
  // }
}
