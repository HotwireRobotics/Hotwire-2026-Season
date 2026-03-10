package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Simple wrapper that auto-selects Xbox vs. PlayStation mappings per port. */
public class HybridGamepad {
  private final CommandXboxController xbox;
  private final CommandPS4Controller ps4;
  private final CommandPS5Controller ps5;
  private final boolean usePlayStation;

  public HybridGamepad(int port) {
    String name = DriverStation.getJoystickName(port).toLowerCase();
    // Common DualShock/DualSense names on Windows/DS: "wireless controller", "dualsense"
    boolean looksPlayStation =
        name.contains("wireless controller")
            || name.contains("dualsense")
            || name.contains("dualshock")
            || name.contains("playstation")
            || name.contains("ps5")
            || name.contains("ps4");

    if (looksPlayStation) {
      // Prefer a PS5-style wrapper when available, fall back to PS4 mapping.
      ps5 = new CommandPS5Controller(port);
      ps4 = new CommandPS4Controller(port);
      xbox = null;
      usePlayStation = true;
    } else {
      xbox = new CommandXboxController(port);
      ps4 = null;
      ps5 = null;
      usePlayStation = false;
    }
  }

  private boolean isPs5() {
    return usePlayStation && ps5 != null;
  }

  private boolean isPs4() {
    return usePlayStation && ps5 == null && ps4 != null;
  }

  // Axes
  public double getLeftY() {
    if (!usePlayStation) return xbox.getLeftY();
    return isPs5() ? ps5.getLeftY() : ps4.getLeftY();
  }

  public double getLeftX() {
    if (!usePlayStation) return xbox.getLeftX();
    return isPs5() ? ps5.getLeftX() : ps4.getLeftX();
  }

  public double getRightX() {
    if (!usePlayStation) return xbox.getRightX();
    return isPs5() ? ps5.getRightX() : ps4.getRightX();
  }

  // POV helpers
  public Trigger povUp() {
    if (!usePlayStation) return xbox.povUp();
    return isPs5() ? ps5.povUp() : ps4.povUp();
  }

  public Trigger povRight() {
    if (!usePlayStation) return xbox.povRight();
    return isPs5() ? ps5.povRight() : ps4.povRight();
  }

  public Trigger povDown() {
    if (!usePlayStation) return xbox.povDown();
    return isPs5() ? ps5.povDown() : ps4.povDown();
  }

  // Face buttons – map Xbox A/X to Cross/Square on PS.
  public Trigger a() {
    if (!usePlayStation) return xbox.a();
    return isPs5() ? ps5.cross() : ps4.cross(); // bottom button
  }

  public Trigger x() {
    if (!usePlayStation) return xbox.x();
    return isPs5() ? ps5.square() : ps4.square(); // left button
  }

  // Triggers
  public Trigger leftTrigger() {
    if (!usePlayStation) return xbox.leftTrigger();
    return isPs5() ? ps5.L2() : ps4.L2();
  }

  public Trigger rightTrigger() {
    if (!usePlayStation) return xbox.rightTrigger();
    return isPs5() ? ps5.R2() : ps4.R2();
  }

  // Rumble
  public void setRumble(RumbleType type, double value) {
    if (!usePlayStation) {
      xbox.getHID().setRumble(type, value);
      return;
    }
    if (isPs5()) {
      ps5.getHID().setRumble(type, value);
    } else if (isPs4()) {
      ps4.getHID().setRumble(type, value);
    }
  }
}
