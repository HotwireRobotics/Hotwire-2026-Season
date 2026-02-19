package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

public class ProtoIntake extends ModularSubsystem implements Systerface {
  // Defining motors
  private final TalonFX rollers;
  private final TalonFX lower;
  private final TalonFX arm;

  public enum Device {
    ROLLERS,
    LOWER,
    ARM
  }
  //SysId routines for rollers and arm
  private final SysIdRoutine m_sysIdRoutineRight;
  private final SysIdRoutine m_sysIdRoutineLeft;
  private final SysIdRoutine m_sysIdRoutineARM;

  public ProtoIntake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    lower = new TalonFX(Constants.MotorIDs.i_follower);
    arm = new TalonFX(99);

    defineDevice(Device.ROLLERS, rollers);
    defineDevice(Device.LOWER, lower);
    defineDevice(Device.ARM, arm);
    /**
     * Configures SysId for inake, left and right
     *
     * <p>Commands of which are at the bottom, both Quasistatic and Dynamic for both
     */
    m_sysIdRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Intake/SysIdState/Right", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.ROLLERS, voltage.in(Volts)), null, this));
    m_sysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Intake/SysIdState/Left", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.ROLLERS, voltage.in(Volts)), null, this));
    // For when we add a 2nd motor to lift up the intake
    m_sysIdRoutineARM =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Intake/SysIdState/Down", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.ARM, voltage.in(Volts)), null, this));
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
    Logger.recordOutput("Intake/RollersArm/Position", arm.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Intake/RollersArm/Velocity", arm.getVelocity().getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Intake/RollersArm/Voltage", arm.getMotorVoltage().getValueAsDouble(), "V");
    Logger.recordOutput(
        "Intake/RollersArm/Current", arm.getSupplyCurrent().getValueAsDouble(), "A");
    Logger.recordOutput(
        "Intake/RollersArm/Temperature", arm.getDeviceTemp().getValueAsDouble(), "°C");

    if (isActiveDevice(Device.ROLLERS)) {
      state = State.INTAKING;
    } else {
      state = State.STOPPED;
    }
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
  /**
   * Allows you to limit the voltage of the intake rollers
   *
   * @param device
   * @param volts
   */
  public void runDeviceVoltage(@NotNull Device device, @NotNull double volts) {
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
  public Command runIntake(@NotNull double speed) {
    return Commands.run(
        () -> {
          runDevice(Device.ROLLERS, speed);
          runDevice(Device.LOWER, speed);
        });
  }

  public Command moveIntake(@NotNull double speed) {
    return Commands.run(
        () -> {
          runDevice(Device.ARM, speed);
        });
  }
  // Detects BrownOut
  double voltage = RobotController.getBatteryVoltage();{
    if (voltage < 9.0) {
      System.out.println("Low voltage warning: " + voltage + "V"); 
    }
  }

  /**
   * Commands mentioned above for m_sysIdRoutineRight and m_sysIdRoutineLeft
   *
   * @param direction
   * @return m_sysIdRoutineRight, m_sysIdRoutineLeft
   */
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

  public Command sysIdQuasistaticARM(@NotNull SysIdRoutine.Direction direction) {
    return m_sysIdRoutineARM.quasistatic(direction);
  }

  public Command sysIdDynamicARM(@NotNull SysIdRoutine.Direction direction) {
    return m_sysIdRoutineARM.dynamic(direction);
  }
}
