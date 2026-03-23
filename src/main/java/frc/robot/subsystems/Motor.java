package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Motor extends TalonFX {

  private final SysIdRoutine sysIdRoutine;
  private final Subsystem subsystem;
  private final TalonFXConfigurator configurator;

  public Motor(Subsystem subsystem, int deviceID, Current limit) {
    super(deviceID);
    // Identify TalonFX configuration.
    configurator = this.getConfigurator();

    // Limit current to 40A to prevent damage to the motor and robot.
    CurrentLimitsConfigs current =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(limit.in(Amps))
            .withSupplyCurrentLimitEnable(true);

    // Apply configuration settings.
    configurator.apply(current);

    // Initialize SysId routine for this motor.
    VoltageOut control = new VoltageOut(0);
    this.subsystem = subsystem;
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) ->
                    Logger.recordOutput(
                        "Motors/SysId/" + super.getDescription(), state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> super.setControl(control.withOutput(voltage.in(Volts))),
                null,
                subsystem));
  }

  /** Get the enclosing subsystem for this motor. */
  public Subsystem getSubsystem() {
    return subsystem;
  }

  /**
   * Log TalonFX data to the Logger. This should be called periodically in the subsystem's periodic
   * method.
   */
  public void log() {
    // Log TalonFX data.
    Logs.log(this);
  }

  /**
   * Run the quasistatic SysId routine for this motor.
   *
   * @param direction
   */
  private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Run the dynamic SysId routine for this motor.
   *
   * @param direction
   */
  private Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Run the all SysId routines in sequence for this motor, including both quasistatic and dynamic
   * tests in both directions.
   */
  public Command runSysId() {
    return new SequentialCommandGroup(
        sysIdQuasistatic(SysIdRoutine.Direction.kForward),
        sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
        sysIdDynamic(SysIdRoutine.Direction.kForward),
        sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Configure motor output modifiers.
   * 
   * @param direction of motion.
   * @param neutral mode.
   */
  public void setDirection(InvertedValue direction, NeutralModeValue neutral) {
    MotorOutputConfigs output = new MotorOutputConfigs();
    
    output.withNeutralMode(neutral);
    output.withInverted(direction);
    configurator.apply(output);
  }

  /**
   * Configure follower target.
   * 
   * @param id
   * @param aligned
   */
  public void setMaster(int id, boolean aligned) {
    setControl(new Follower(id, aligned ? 
        MotorAlignmentValue.Aligned : MotorAlignmentValue.Opposed));
  }

  /**
   * Configure follower target from primary motor.
   * 
   * @param motor
   * @param aligned
   */
  public void setMaster(Motor motor, boolean aligned) {
    setMaster(motor.getDeviceID(), aligned);
  }
}