package frc.robot.subsystems.indication;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class LuminalIndicators extends SubsystemBase {

  private final IndicatorsIO io;
  private final IndicatorsIOInputsAutoLogged inputs = new IndicatorsIOInputsAutoLogged();

  private enum Event {
    AUTONOMOUS,
    ACTIVE,
    INACTIVE,
    ACTIVE,
    INACTIVE,
  }


  private final HashMap<Event, ControlRequest> color;
  private final Timer timer = new Timer();
  private Time time = Seconds.of(0);

  private Supplier<SolidColor> toggle(SolidColor t, SolidColor f, Frequency frequency) {
    return () -> (((Math.floor(time.in(Seconds) * frequency.in(Hertz)) % 2) == 1) ? t : f);
  }

  /** Constructs indicators with selected IO implementation. */
  public LuminalIndicators(IndicatorsIO io) {
    this.io = io;

    // Configuration
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;

    color = new HashMap<>();
    color.put(Event.ACTIVE, toggle(Constants.Indication.LEDColor(0, 255, 0), Constants.Indication.LEDColor(0, 0, 0), Hertz.of(0.667)));
    color.put(Event.INACTIVE, () -> Constants.Indication.LEDColor(255, 0, 0));
    color.put(Event.AUTONOMOUS, () -> Constants.Indication.LEDColor(20, 20, 150));

    timer.start();
  }

  public void time() {
    timer.reset();
    timer.start();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indicators", inputs);

    Time t = Seconds.of(DriverStation.getMatchTime());
    Time length = (DriverStation.isAutonomous()) ? Constants.autoLength : Constants.teleopLength;

    // Get period-relative time.
    time = (t.isEquivalent(Seconds.of(-1))) ? Seconds.of(timer.get()) : length.minus(t);
    Boolean b = (Math.floor(time.in(Seconds)) % 2) == 1;

    indicatorPipeline(time, b);
  }

  private Supplier<SolidColor> getRequest(Event event) {
    return color.get(event);
  }

  public void indicatorPipeline(Time time, boolean toggle) {
    Logger.recordOutput("Indicators/time", time);

    if (DriverStation.isAutonomous()) {
      updateLEDs(getRequest(Event.AUTONOMOUS).get());
    } else {
      if (Constants.Indication.autonomousVictory()) {
        updateLEDs(getRequest(Event.INACTIVE).get());
      } else {
        updateLEDs(getRequest(Event.ACTIVE).get());
      }
    }
  }

  private void updateLEDs(ControlRequest color) {
    io.setControl(color);
  }
}
