package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoIntake extends ModularSubsystem implements Systerface {

  public final TalonFX rollers;
  public final TalonFX arm;
  public final Slot0Configs slot;

  private final PositionVoltage m_PositionVoltage;

  public enum Device {
    ROLLERS,
    ARM
  }

  // private final SysIdRoutine m_sysIdRoutineRight;
  // private final SysIdRoutine m_sysIdRoutineLeft;
  // private final SysIdRoutine m_sysIdRoutineARM;

  public ProtoIntake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    arm = new TalonFX(Constants.MotorIDs.i_arm);
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.withSupplyCurrentLimit(40);
    arm.getConfigurator().apply(currentLimits);
    defineDevice(new DevicePointer(Device.ROLLERS, rollers), new DevicePointer(Device.ARM, arm));

    m_PositionVoltage = new PositionVoltage(Degrees.of(0));

    slot = new Slot0Configs();
    slot.withKP(2.0);

    // Configuration
    arm.setControl(m_PositionVoltage);
    arm.getConfigurator().apply(slot);
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
    Logger.recordOutput("Intake/Rollers/Position", rollers.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Intake/Rollers/Velocity", rollers.getVelocity().getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Intake/Rollers/Voltage", rollers.getMotorVoltage().getValueAsDouble(), "V");
    Logger.recordOutput(
        "Intake/Rollers/Current", rollers.getSupplyCurrent().getValueAsDouble(), "A");
    Logger.recordOutput(
        "Intake/Rollers/Temperature", rollers.getDeviceTemp().getValueAsDouble(), "°C");

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
        });
  }

  public Command pointArm(Angle angle) {
    return Commands.runOnce(() -> arm.setControl(m_PositionVoltage.withPosition(angle)));
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
