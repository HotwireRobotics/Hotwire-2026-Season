package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public static class Scaler {
    // Declare a key and default value for the scaler.
    private final String key;
    private final double def;

    public Scaler(String key, double defaultValue) {
      this.key = key;
      this.def = defaultValue;

      // set default once.
      SmartDashboard.putNumber(key, defaultValue);
    }

    /** Get the current value of the scaler from the SmartDashboard. */
    public double get() {
      return SmartDashboard.getNumber(key, def);
    }
  }

  // Initialize suppliers for dashboard values.
  public static final Toggle visionEnabled = new Toggle("Dashboard/Limelight Vision", true);
  public static final Toggle alignmentRequirement =
      new Toggle("Dashboard/Alignment Requirement", true);
  public static final Scaler x = new Scaler("Dashboard/Pose/x", 0);
  public static final Scaler y = new Scaler("Dashboard/Pose/y", 0);
}
