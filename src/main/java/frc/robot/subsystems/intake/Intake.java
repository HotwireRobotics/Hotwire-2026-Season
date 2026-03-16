package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import frc.robot.constants.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends ModularSubsystem implements Systerface {

  public final TalonFX rollers;
  public final TalonFX wrist;
  private final PositionVoltage m_PositionVoltage;
  private final Slot0Configs slot;

  public enum Device {
    ROLLERS,
    WRIST
  }

  public enum ArmState {
    FORWARD,
    BACKWARD,
    ZERO
  }

  private ArmState armState = ArmState.ZERO;

  public Intake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    wrist = new TalonFX(Constants.MotorIDs.i_arm);
    defineDevice(new DevicePointer(Device.ROLLERS, rollers), new DevicePointer(Device.WRIST, wrist));

    m_PositionVoltage = new PositionVoltage(Degrees.of(0));

    slot = new Slot0Configs();
    configureProportional(18.0);

    // Configuration
    wrist.setControl(m_PositionVoltage);
    wrist.getConfigurator().apply(slot);
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

    if (volts == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }

  public Command runIntake() {
    return runDevice(Device.ROLLERS, Constants.Intake.kSpeed);
  }

  public Command runIntake(Supplier<Boolean> inverse) {
    return runDevice(Device.ROLLERS, () -> (Constants.Hopper.kSpeed * (inverse.get() ? -1 : 1)));
  }

  public Command stopIntake() {
    return runDevice(Device.ROLLERS, 0);
  }

  public Command controlArm(ArmState state) {
    return Commands.runOnce(
        () -> {
          armState = state;
        });
  }

  public Command oscillateArm(Angle angle, Frequency frequency) {
    Time period = Seconds.of(1 / (2 * frequency.in(Hertz)));
    RepeatCommand group =
        new SequentialCommandGroup(
                lowerArm(), Commands.waitTime(period),
                raiseArm(angle), Commands.waitTime(period))
            .repeatedly();
    group.addRequirements(this);
    return group;
  }

  public Command raiseArm(Angle angle) {
    return Commands.runOnce(
        () -> {
          configureProportional(18.0);
          wrist.setControl(m_PositionVoltage.withPosition(angle));
        });
  }

  public Command lowerArm() {
    return Commands.runOnce(
        () -> {
          configureProportional(18.0);
          wrist.setControl(m_PositionVoltage.withPosition(Degrees.of(0)));
        });
  }

  public Command emergency() {
    return Commands.runOnce(
        () -> {
          configureProportional(28.0);
          wrist.setControl(m_PositionVoltage.withPosition(Degrees.of(90)));
        });
  }
}
