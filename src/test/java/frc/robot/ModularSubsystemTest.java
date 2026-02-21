package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

/** Unit tests for null-safe ModularSubsystem contracts. */
class ModularSubsystemTest {
  @Test
  void getDevicesReturnsEmptyArrayForUnknownKey() {
    ModularSubsystem<String> subsystem = new ModularSubsystem<>();
    assertEquals(0, subsystem.getDevices("missing").length);
  }

  @Test
  void activeChecksAreSafeForUnknownKey() {
    ModularSubsystem<String> subsystem = new ModularSubsystem<>();
    assertDoesNotThrow(() -> subsystem.specifyActiveDevice("unknown"));
    assertFalse(subsystem.isActiveDevice("unknown"));
  }
}
