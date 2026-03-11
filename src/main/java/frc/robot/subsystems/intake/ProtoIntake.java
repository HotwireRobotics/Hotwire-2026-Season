package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem with rollers and arm. Implements Systerface for device/sysid integration. */
public class ProtoIntake extends edu.wpi.first.wpilibj2.command.SubsystemBase implements Systerface {

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

  /** Command for running intake rollers at open-loop speed. */
  public Command runIntake(double speed) {
    return Commands.runOnce(() -> runDevice(Device.ROLLERS, speed));
  }

  /** Command to set arm manual voltage state. */
  public Command controlArm(ArmState state) {
    return Commands.runOnce(() -> armState = state);
  }

  /** Oscillates the arm up and down at a chosen frequency. */
  public Command occilateArm(Frequency frequency) {
    return new SequentialCommandGroup(
            lowerArm(), Commands.waitSeconds(1 / frequency.in(Hertz)), raiseArm())
        .repeatedly();
  }

  /** Position command for arm-up. */
  public Command raiseArm() {
    return Commands.runOnce(
        () -> {
          io.configureArmPositionKp(18.0);
          io.setArmPositionDegrees(60.0);
        });
  }

  /** Position command for arm-down. */
  public Command lowerArm() {
    return Commands.runOnce(
        () -> {
          io.configureArmPositionKp(18.0);
          io.setArmPositionDegrees(0.0);
        });
  }

  /** Position command for emergency arm-up. */
  public Command emergency() {
    return Commands.runOnce(
        () -> {
          io.configureArmPositionKp(28.0);
          io.setArmPositionDegrees(90.0);
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
