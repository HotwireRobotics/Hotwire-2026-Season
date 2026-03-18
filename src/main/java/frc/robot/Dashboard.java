package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Toggle {
    // Declare a key and default value for the toggle.
    private final String key;
    private final boolean def;
  
    public Toggle(String key, boolean defaultValue) {
      this.key = key;
      this.def = defaultValue;
  
      // set default once.
      SmartDashboard.putBoolean(key, defaultValue);
    }
  
    /**
     * Get the current value of the toggle from the SmartDashboard.
     */
    public boolean get() {
      return SmartDashboard.getBoolean(key, def);
    }
  }

public class Dashboard {
    // Initialize suppliers for dashboard values.
    public static final Toggle visionEnabled =
        new Toggle("Dashboard/Limelight Vision", true);
    public static final Toggle alignmentRequirement =
        new Toggle("Dashboard/Alignment Requirement", true);
}
