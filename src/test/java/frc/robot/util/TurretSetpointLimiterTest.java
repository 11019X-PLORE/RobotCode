package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TurretSetpointLimiterTest {
  private static final double EPSILON = 1e-9;

  @Test
  void limitsVelocityAndAccelerationWhenRequestedVelocityIsTooHigh() {
    TurretSetpointLimiter limiter = new TurretSetpointLimiter();
    limiter.reset(0.0, 0.0);

    TurretSetpointLimiter.State state = limiter.calculate(10.0, 100.0, 2.0, 5.0, 0.02);

    assertTrue(Math.abs(state.velocityRadPerSec()) <= 0.1 + EPSILON);
    assertTrue(Math.abs(state.accelerationRadPerSecSq()) <= 5.0 + EPSILON);
  }

  @Test
  void followsMovingGoalWhenAlreadyAtGoalState() {
    TurretSetpointLimiter limiter = new TurretSetpointLimiter();
    limiter.reset(0.0, 1.0);

    TurretSetpointLimiter.State state = limiter.calculate(0.0, 1.0, 3.0, 10.0, 0.02);

    assertEquals(0.02, state.positionRad(), EPSILON);
    assertEquals(1.0, state.velocityRadPerSec(), EPSILON);
    assertEquals(0.0, state.accelerationRadPerSecSq(), EPSILON);
  }

  @Test
  void usesProfiledAccelerationInsteadOfRawRequestedAcceleration() {
    TurretSetpointLimiter limiter = new TurretSetpointLimiter();
    limiter.reset(0.0, 0.0);

    TurretSetpointLimiter.State state = limiter.calculate(-10.0, -100.0, 2.0, 4.0, 0.02);

    assertEquals(-4.0, state.accelerationRadPerSecSq(), EPSILON);
  }

  @Test
  void brakesWhenMovingGoalApproachesCurrentPosition() {
    TurretSetpointLimiter limiter = new TurretSetpointLimiter();
    limiter.reset(0.0, 0.0);

    TurretSetpointLimiter.State state = limiter.calculate(0.04, -1.0, 3.0, 10.0, 0.02);

    assertTrue(state.velocityRadPerSec() < 0.0);
    assertEquals(-10.0, state.accelerationRadPerSecSq(), EPSILON);
  }
}
