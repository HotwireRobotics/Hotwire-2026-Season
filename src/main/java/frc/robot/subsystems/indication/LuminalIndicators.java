package frc.robot.subsystems.indication;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class LuminalIndicators extends SubsystemBase {

  private final IndicatorsIO io;
  private final IndicatorsIOInputsAutoLogged inputs = new IndicatorsIOInputsAutoLogged();

  private enum Event {
    AUTONOMOUS,
    ACTIVE,
    INACTIVE,
  }

  private final HashMap<Event, ControlRequest> color;

  /** Constructs indicators with selected IO implementation. */
  public LuminalIndicators(IndicatorsIO io) {
    this.io = io;
    color = new HashMap<>();
    color.put(Event.ACTIVE, Constants.Indication.LEDColor(180, 255, 180));
    color.put(Event.INACTIVE, Constants.Indication.LEDColor(255, 190, 180));
    color.put(Event.AUTONOMOUS, Constants.Indication.LEDColor(0, 200, 0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indicators", inputs);

    Time t = Seconds.of(DriverStation.getMatchTime());
    Time length = (DriverStation.isAutonomous()) ? Constants.autoLength : Constants.teleopLength;

    // Get period-relative time.
    Time time = (t.isEquivalent(Seconds.of(-1))) ? Seconds.of(0) : length.minus(t);

    indicatorPipeline();
  }

  public void indicatorPipeline() {}

  public Command updateLEDs(SolidColor color) {
    return runOnce(
        () -> {
          io.setControl(color);
        });
  }
}
