package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoIntake extends ModularSubsystem implements Systerface {

  public final TalonFX rollers;
  public final TalonFX arm;

  public enum Device {
    ROLLERS,
    ARM
  }

  public enum ArmState {
    FORWARD,
    BACKWARD,
    ZERO
  }

  private ArmState armState = ArmState.ZERO;

  public ProtoIntake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    arm = new TalonFX(Constants.MotorIDs.i_arm);
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.withSupplyCurrentLimit(40);
    arm.getConfigurator().apply(currentLimits);
    defineDevice(new DevicePointer(Device.ROLLERS, rollers), new DevicePointer(Device.ARM, arm));
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

    // TODO make more readable
    arm.setControl(
        new VoltageOut(
            armState.equals(ArmState.FORWARD)
                ? Constants.Intake.kArmVolts
                : (armState.equals(ArmState.BACKWARD)
                    ? Constants.Intake.kArmVolts.times(-1)
                    : Volts.of(0))));
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

  public Command moveIntake() {
    return Commands.run(() -> armState = ArmState.BACKWARD)
        .withTimeout(Constants.Intake.targetSecUp)
        .andThen(
            Commands.run(() -> armState = ArmState.FORWARD)
                .withTimeout(Constants.Intake.targetSecDown))
        .andThen(() -> armState = ArmState.ZERO);
  }

  boolean targetPos = false;

  public Command controlArm(ArmState state) {
    return Commands.runOnce(
        () -> {
          armState = state;
        });
      
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
      }}