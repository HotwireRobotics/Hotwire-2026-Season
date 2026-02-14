package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoIntake extends ModularSubsystem implements Systerface {

  private final TalonFX rollers;
  private final TalonFX lower;

  public enum Device {
    ROLLERS,
    LOWER
  }

  private final SysIdRoutine m_sysIdRoutineRight;
  private final SysIdRoutine m_sysIdRoutineLeft;

  public ProtoIntake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    lower = new TalonFX(Constants.MotorIDs.i_follower);

    defineDevice(Device.ROLLERS, rollers);
    defineDevice(Device.LOWER, lower);
    /**
     * 
     * Configures SysId for inake, left and right
     * 
     * Commands of which are at the bottom, both Quasistatic and Dynamic for both
     * 
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
     * 
     * Logs position of (rot)
     * 
     * Logs velocity in (rpm)
     * 
     * Logs voltage current
     * 
     * Logs temp with unit metadata
     * 
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
   * 
   * Allows you to limit the voltage of the intake rollers
   * 
   * @param device
   * 
   * @param volts
   * 
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
   * 
   * Allows you to limit the speed of the intake rollers
   * 
   * @param speed
   * 
   * @return
   * 
   */
  public Command runMechanism(double speed) {
    return Commands.run(
        () -> {
          runDevice(Device.ROLLERS, speed);
          runDevice(Device.LOWER, speed);
        });
  }
  /**
   * 
   * Commands mentioned above for m_sysIdRoutineRight and m_sysIdRoutineLeft
   * 
   * @param direction
   * 
   * @return m_sysIdRoutineRight, m_sysIdRoutineLeft
   */
  public Command sysIdQuasistaticRight(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.quasistatic(direction);
  }  
  public Command sysIdDynamicRight(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.dynamic(direction);
  }
  public Command sysIdQuasistaticLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.quasistatic(direction);
  }

  public Command sysIdDynamicLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.dynamic(direction);
  }
}
