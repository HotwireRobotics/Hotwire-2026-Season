package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Systerface;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Logs;
import frc.robot.subsystems.ModularSubsystem;
import frc.robot.subsystems.Motor;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends ModularSubsystem implements Systerface {

  public final Motor feeder;
  public final Motor left;
  public final Motor right;

  private final VelocityVoltage velControl = new VelocityVoltage(0);

  private final Slot0Configs leftSlot = new Slot0Configs();
  private final Slot0Configs rightSlot = new Slot0Configs();
  private final Slot0Configs feedSlot = new Slot0Configs();

  private final Supplier<AngularVelocity> velocity;

  // Declare device enum.
  public enum Device {
    FEEDER,
    RIGHT,
    LEFT
  }

  private final Debouncer debouncer = new Debouncer(Constants.Shooter.kDebounce.in(Seconds));

  public Shooter(Supplier<AngularVelocity> velocity) {

    this.velocity = velocity;

    // Initialize devices.
    left = new Motor(this, Constants.MotorIDs.s_shooterL, Amps.of(60));
    left.setDirection(InvertedValue.CounterClockwise_Positive, NeutralModeValue.Coast);

    right = new Motor(this, Constants.MotorIDs.s_shooterR, Amps.of(60));
    right.setDirection(InvertedValue.Clockwise_Positive, NeutralModeValue.Coast);

    feeder = new Motor(this, Constants.MotorIDs.s_feeder, Amps.of(40));
    feeder.setDirection(InvertedValue.Clockwise_Positive, NeutralModeValue.Coast);

    // Define devices.
    defineDevice(
      new DevicePointer(Device.RIGHT, right),
      new DevicePointer(Device.LEFT,  left),
      new DevicePointer(Device.FEEDER, feeder)
    );

    leftSlot.withKV(0.12009).withKS(0.24998).withKP(0.8);
    rightSlot.withKV(0.11965).withKS(0.34220).withKP(0.8);
    feedSlot.withKV(0.12009).withKS(0.24998).withKP(0.8);

    configureControl();

    // var file = Filesystem.getDeployDirectory()
    //     .toPath()
    //     .resolve("shooter/config.json");

    // try {
    //     var data = new ObjectMapper().readTree(file.toFile());
    //     double rpm = data.get("rpm").asDouble();
    // } catch (IOException e) {
    //     e.printStackTrace();
    // }
  }

  private enum State {
    STOPPED,
    FIRING
  }

  private State state = State.STOPPED;

  @Override
  public Object getState() {
    return state;
  }

  /** True while the flywheel command is running. The fuel sim launches only in this state. */
  public boolean isFiring() {
    return state == State.FIRING;
  }

  public void setState(State newState) {
    state = newState;
  }

  @Override
  public void periodic() {
    // Keep the flywheel on the live supplier for the whole time we are firing, so distance and
    // chassis speed change the setpoint every cycle instead of only when the command restarts.
    if (state == State.FIRING) {
      applyTrackedVelocity();
    }

    logDevices();

    Logs.log(this, state);
  }

  /**
   * Latest supplier value, with a finite magnitude at or below {@link Constants.Shooter#kMaxRpm}.
   * A broken supplier returns the ferry speed rather than NaN or a runaway setpoint.
   */
  private AngularVelocity readVelocity() {
    try {
      AngularVelocity target = velocity.get();
      if (target == null) {
        return Constants.Shooter.kSpeed;
      }
      double rpm = target.in(RPM);
      if (!Double.isFinite(rpm)) {
        return Constants.Shooter.kSpeed;
      }
      double magnitude = Math.min(Math.abs(rpm), Constants.Shooter.kMaxRpm);
      return RPM.of(Math.copySign(magnitude, rpm));
    } catch (RuntimeException ex) {
      Logger.recordOutput("Shooter/VelocityFault", ex.toString());
      return Constants.Shooter.kSpeed;
    }
  }

  /** Push the current setpoint to the flywheels and feeder. */
  private void applyTrackedVelocity() {
    AngularVelocity target = readVelocity();
    Logger.recordOutput("Shooter/CommandedRPM", target.in(RPM));
    if (Math.abs(target.in(RPM)) < 1.0) {
      applyPercent(0.0, left, right, feeder);
      return;
    }
    applyVelocity(target, left, right, feeder);
  }

  private void applyVelocity(AngularVelocity velocity, Motor... motors) {
    for (var m : motors) m.setControl(velControl.withVelocity(velocity));
  }

  private void applyPercent(double percent, Motor... motors) {
    for (var m : motors) m.set(percent);
  }

  public void start() {
    setState(State.FIRING);
    applyTrackedVelocity();
  }

  public void stall() {
    applyPercent(Constants.Shooter.kZero.in(RPM), left, right, feeder);

    setState(State.STOPPED);
  }

  public boolean isReady() {
    AngularVelocity target = readVelocity();
    try {
      return debouncer.calculate(
          left.getVelocity().getValue().isNear(target, Constants.Shooter.kVelocityTolerance)
              && right.getVelocity().getValue().isNear(target, Constants.Shooter.kVelocityTolerance));
    } catch (RuntimeException ex) {
      return false;
    }
  }

  public Command run() {
    return runOnce(() -> start());
  }

  public Command halt() {
    return runOnce(() -> stall());
  }

  private void configureControl() {
    left.getConfigurator().apply(leftSlot);
    right.getConfigurator().apply(rightSlot);
    feeder.getConfigurator().apply(feedSlot);
  }

  public Command sysIdRightAnalysis() {
    return right.runSysId();
  }

  public Command sysIdLeftAnalysis() {
    return left.runSysId();
  }

  public Command sysIdFeederAnalysis() {
    return feeder.runSysId();
  }
}
