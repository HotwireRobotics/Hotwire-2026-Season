package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

  public Shooter(Supplier<AngularVelocity> velocity) {

    this.velocity = velocity;

    left = new Motor(this, Constants.MotorIDs.s_shooterL, Amps.of(60));
    right = new Motor(this, Constants.MotorIDs.s_shooterR, Amps.of(60));
    feeder = new Motor(this, Constants.MotorIDs.s_feeder, Amps.of(40));

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

  public void setState(State newState) {
    state = newState;
  }

  @Override
  public void periodic() {

    left.log();
    right.log();
    feeder.log();

    Logs.log(this, state);
  }

  private void applyVelocity(AngularVelocity velocity, Motor... motors) {
    for (var m : motors) m.setControl(velControl.withVelocity(velocity));
  }

  public void start() {
    applyVelocity(velocity.get(), left, right, feeder);

    setState(State.FIRING);
  }

  public void stall() {
    applyVelocity(Constants.Shooter.kZero, left, right, feeder);

    setState(State.STOPPED);
  }

  public boolean isReady() {
    return left.getVelocity()
            .getValue()
            .isNear(velocity.get(), Constants.Shooter.kVelocityTolerance)
        && right
            .getVelocity()
            .getValue()
            .isNear(velocity.get(), Constants.Shooter.kVelocityTolerance);
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
