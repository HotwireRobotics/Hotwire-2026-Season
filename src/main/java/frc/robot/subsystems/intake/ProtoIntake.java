package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoIntake extends SubsystemBase implements Systerface {

  private TalonFX rollers;

  public ProtoIntake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    // follower = new TalonFX(Constants.MotorIDs.i_follower);

    // Follower motor for rollers.
    // follower.setControl(new Follower(Constants.MotorIDs.i_rollers, MotorAlignmentValue.Aligned));
  }

  private enum State {
    STOPPED,
    HALTED,
    INTAKING // Running rollers
  }

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/State", state.toString());
    Logger.recordOutput("Intake/RollersVelocity", rollers.getVelocity().getValue());
    Logger.recordOutput("Intake/RollersCurrent", rollers.getSupplyCurrent().getValue());
  }

  public Command runRollersConstantCommand(double speed) {
    return Commands.run(
        () -> {
          rollers.set(speed);
        });
  }

  public void setRollers(double speed) {
    rollers.set(speed);
  }

  public void setVoltage(double volt) {
    rollers.setVoltage(volt);
  }

  public Command setVoltageCommand(double volt) {
    return Commands.run(
        () -> {
          rollers.setVoltage(volt);
        });
  }

  public void stopRollers() {
    rollers.set(0);
  }

  public Command stopRollersCommand() {
    return Commands.run(
        () -> {
          rollers.set(0);
        });
  }
}
