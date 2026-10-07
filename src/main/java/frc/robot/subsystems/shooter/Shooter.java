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

  public enum State {
    STOPPED,
    FIRING
  }

  private State state = State.STOPPED;

  @Override
  public Object getState() {
    return state;
  }

  public void setState(State newState) {
    state = newState;
  }

  @Override
  public void periodic() {
    // Push the live supplier every cycle while firing. Distance and chassis speed change the
    // setpoint continuously, not only when the run-once command starts.
    if (state == State.FIRING) {
      applyVelocity(readVelocity(), left, right, feeder);
    }

    logDevices();

    Logs.log(this, state);
  }

  /**
   * Latest supplier value, with a finite magnitude at or below {@link Constants.Shooter#kMaxRpm}.
   * A broken supplier returns the ferry speed rather than NaN or a runaway setpoint. Sign is kept
   * so inverse still spins the flywheel backward.
   */
  private AngularVelocity readVelocity() {
    try {
      AngularVelocity target = velocity.get();
      if (target == null || !Double.isFinite(target.in(RPM))) {
        return Constants.Shooter.kSpeed;
      }
      double rpm =
          Math.copySign(
              Math.min(Math.abs(target.in(RPM)), Constants.Shooter.kMaxRpm), target.in(RPM));
      return RPM.of(rpm);
    } catch (RuntimeException ex) {
      return Constants.Shooter.kSpeed;
    }
  }

  private void applyVelocity(AngularVelocity velocity, Motor... motors) {
    for (var m : motors) m.setControl(velControl.withVelocity(velocity));
  }

  private void applyPercent(double percent, Motor... motors) {
    for (var m : motors) m.set(percent);
  }

  public void start() {
    AngularVelocity commanded = readVelocity();
    if (Math.abs(commanded.in(RPM)) < 1.0) {
      applyPercent(Constants.Shooter.kZero.in(RPM), left, right, feeder);

      setState(State.STOPPED);
    } else {
      applyVelocity(commanded, left, right, feeder);

      setState(State.FIRING);
    }
  }

  public void stall() {
    applyPercent(Constants.Shooter.kZero.in(RPM), left, right, feeder);

    setState(State.STOPPED);
  }

  public boolean isReady() {
    AngularVelocity target = readVelocity();
    return debouncer.calculate(
        left.getVelocity().getValue().isNear(target, Constants.Shooter.kVelocityTolerance) &&
        right.getVelocity().getValue().isNear(target, Constants.Shooter.kVelocityTolerance));
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
