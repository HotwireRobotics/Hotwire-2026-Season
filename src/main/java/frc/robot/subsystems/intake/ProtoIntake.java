package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem with rollers and arm. Implements Systerface for device/sysid integration. */
public class ProtoIntake extends edu.wpi.first.wpilibj2.command.SubsystemBase implements Systerface {

  public final TalonFX rollers;
  public final TalonFX arm;
  private final PositionVoltage m_PositionVoltage;
  private final Slot0Configs slot;
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean rollersActive = false;

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
  /** Constructs intake with the selected IO implementation. */
  public ProtoIntake(IntakeIO io) {
    this.io = io;
    io.configureArmPositionKp(18.0);
  }

  private enum State {
    STOPPED,
    INTAKING
  }

  private State state = State.STOPPED;

  @Override
  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

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
    state = rollersActive ? State.INTAKING : State.STOPPED;
    Logger.recordOutput("Intake/State", state.toString());

    final double armVolts =
        armState.equals(ArmState.FORWARD)
            ? Constants.Intake.kArmVolts.in(edu.wpi.first.units.Units.Volts)
            : (armState.equals(ArmState.BACKWARD)
                ? -Constants.Intake.kArmVolts.in(edu.wpi.first.units.Units.Volts)
                : 0.0);
    io.setArmVoltage(armVolts);
  }

  /** Device open-loop control helper. */
  public void runDevice(Device device, double speed) {
    switch (device) {
      case ROLLERS -> {
        io.setRollersOpenLoop(speed);
        rollersActive = speed != 0.0;
      }
      case ARM -> io.setArmVoltage(
          speed * Constants.Intake.kArmVolts.in(edu.wpi.first.units.Units.Volts));
    }
  }

  /** Device voltage control helper. */
  public void runDeviceVoltage(Device device, double volts) {
    switch (device) {
      case ROLLERS -> {
        io.setRollersOpenLoop(volts / 12.0);
        rollersActive = volts != 0.0;
      }
      case ARM -> io.setArmVoltage(volts);
    }
  }

  public Command runIntake() {
    return runDevice(Device.ROLLERS, Constants.Intake.kSpeed);
  }

  public Command stopIntake() {
    return runDevice(Device.ROLLERS, 0);
  }

  /** Command to set arm manual voltage state. */
  public Command controlArm(ArmState state) {
    return Commands.runOnce(() -> armState = state);
  }

  /** Oscillates the arm up and down at a chosen frequency. */
  public Command occilateArm(Frequency frequency) {
    Time period = Seconds.of(1 / (2 * frequency.in(Hertz)));
    return new SequentialCommandGroup(
            lowerArm(), Commands.waitTime(period),
            raiseArm(), Commands.waitTime(period))
        .repeatedly();
  }

  /** Position command for arm-up. */
  public Command raiseArm() {
    return Commands.runOnce(
        () -> {
          configureProportional(18.0);
          arm.setControl(m_PositionVoltage.withPosition(Degrees.of(60)));
        });
  }

  /** Position command for arm-down. */
  public Command lowerArm() {
    return Commands.runOnce(
        () -> {
          configureProportional(18.0);
          arm.setControl(m_PositionVoltage.withPosition(Degrees.of(0)));
        });
  }

  /** Position command for emergency arm-up. */
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
