package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem with rollers and arm. Implements Systerface for device/sysid integration. */
public class Intake extends ModularSubsystem implements Systerface {

  public final TalonFX rollers;
  public final TalonFX arm;

  private PositionVoltage m_PositionVoltage;
  private Slot0Configs slot;

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

  public Intake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    arm = new TalonFX(Constants.MotorIDs.i_arm);
    // CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    // currentLimits.withSupplyCurrentLimit(40);
    // arm.getConfigurator().apply(currentLimits);
    defineDevice(new DevicePointer(Device.ROLLERS, rollers), new DevicePointer(Device.ARM, arm));

    m_PositionVoltage = new PositionVoltage(Degrees.of(0));

    slot = new Slot0Configs();
    configureProportional(18.0);

    // Configuration
    arm.setControl(m_PositionVoltage);
    arm.getConfigurator().apply(slot);
  }

  public void configureProportional(double kP) {
    slot.withKP(kP);
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

  public Command controlArm(ArmState state) {
    return Commands.runOnce(
        () -> {
          armState = state;
        });
  }

  public Command occilateArm(Frequency frequency) {
    return new SequentialCommandGroup(
            lowerArm(), Commands.waitSeconds(1 / frequency.in(Hertz)), raiseArm())
        .repeatedly();
  }

  public Command raiseArm() {
    return Commands.runOnce(
        () -> {
          configureProportional(18.0);
          arm.setControl(m_PositionVoltage.withPosition(Degrees.of(60)));
        });
  }

  public Command lowerArm() {
    return Commands.runOnce(
        () -> {
          configureProportional(18.0);
          arm.setControl(m_PositionVoltage.withPosition(Degrees.of(0)));
        });
  }

  public Command emergency() {
    return Commands.runOnce(
        () -> {
          configureProportional(28.0);
          arm.setControl(m_PositionVoltage.withPosition(Degrees.of(90)));
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
