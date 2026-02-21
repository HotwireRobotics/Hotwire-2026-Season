package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Unit tests for safe constant and shooter-regression behavior. */
class ConstantsTest {
  @Test
  void isRedAllianceIsAlwaysSafeToCall() {
    assertDoesNotThrow(Constants::isRedAlliance);
  }

  @Test
  void regressClampsNegativeDistanceToZeroRpm() {
    double rpm = Constants.regress(Meters.of(-2.0)).in(RPM);
    assertEquals(0.0, rpm, 1e-9);
  }

  @Test
  void shooterRegressionRejectsInvalidUpdates() {
    double originalBase = Constants.ShooterRegression.getBaseRpm();
    double originalExponent = Constants.ShooterRegression.getDistanceExponent();
    try {
      Constants.ShooterRegression.update(2000.0, 0.7);
      assertEquals(2000.0, Constants.ShooterRegression.getBaseRpm(), 1e-9);
      assertEquals(0.7, Constants.ShooterRegression.getDistanceExponent(), 1e-9);

      Constants.ShooterRegression.update(-5.0, 0.7);
      assertEquals(2000.0, Constants.ShooterRegression.getBaseRpm(), 1e-9);
      assertEquals(0.7, Constants.ShooterRegression.getDistanceExponent(), 1e-9);

      Constants.ShooterRegression.update(2000.0, 0.0);
      assertTrue(Constants.ShooterRegression.getDistanceExponent() > 0.0);
    } finally {
      Constants.ShooterRegression.update(originalBase, originalExponent);
    }
  }
}
