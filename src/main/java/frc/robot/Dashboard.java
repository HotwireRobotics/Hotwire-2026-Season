package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

public class Dashboard {

  public static class Toggle {
    // Declare a key and default value for the toggle.
    private final String key;
    private final boolean def;

    public Toggle(String key, boolean defaultValue) {
      this.key = key;
      this.def = defaultValue;

      // set default once.
      SmartDashboard.putBoolean(key, defaultValue);
    }

    /** Get the current value of the toggle from the SmartDashboard. */
    public boolean get() {
      return SmartDashboard.getBoolean(key, def);
    }
  }
  /** Live numeric dashboard value with a fixed default. */
  public static class Number {
    private final String key;
    private final double def;

    public Number(String key, double defaultValue) {
      this.key = key;
      this.def = defaultValue;
      SmartDashboard.putNumber(key, defaultValue);
    }

    /** Current value, or the default if the key has not been set. */
    public double get() {
      return SmartDashboard.getNumber(key, def);
    }
  }

  // Initialize suppliers for dashboard values.
  public static final Toggle visionEnabled = new Toggle("Dashboard/Limelight Vision", true);
  public static final Toggle alignmentRequirement =
      new Toggle("Dashboard/Alignment Requirement", true);
  /** Override for {@link Constants.Shooter#kLeadGain}. */
  public static final Number leadGain = new Number("Shooter/Lead Gain", Constants.Shooter.kLeadGain);
}
