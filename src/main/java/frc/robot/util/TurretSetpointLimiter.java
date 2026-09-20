package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class TurretSetpointLimiter {
  private MovingTargetTrapezoidProfile.State previousState =
      new MovingTargetTrapezoidProfile.State(0.0, 0.0);
  private boolean initialized = false;

  public record State(
      double positionRad, double velocityRadPerSec, double accelerationRadPerSecSq) {}

  public void reset(double positionRad, double velocityRadPerSec) {
    previousState = new MovingTargetTrapezoidProfile.State(positionRad, velocityRadPerSec);
    initialized = true;
  }

  public State calculate(
      double goalPositionRad,
      double goalVelocityRadPerSec,
      double maxVelocityRadPerSec,
      double maxAccelerationRadPerSecSq,
      double loopPeriodSecs) {
    if (!initialized) {
      reset(goalPositionRad, 0.0);
    }

    double safeMaxVelocity = Math.max(0.0, Math.abs(maxVelocityRadPerSec));
    double safeMaxAcceleration = Math.max(0.0, Math.abs(maxAccelerationRadPerSecSq));
    double safeLoopPeriod = Math.max(1e-9, loopPeriodSecs);

    MovingTargetTrapezoidProfile profile =
        new MovingTargetTrapezoidProfile(
            new MovingTargetTrapezoidProfile.Constraints(safeMaxVelocity, safeMaxAcceleration));
    MovingTargetTrapezoidProfile.State goal =
        new MovingTargetTrapezoidProfile.State(
            goalPositionRad,
            MathUtil.clamp(goalVelocityRadPerSec, -safeMaxVelocity, safeMaxVelocity));
    MovingTargetTrapezoidProfile.State nextState =
        profile.calculate(safeLoopPeriod, previousState, goal);
    double accelerationRadPerSecSq =
        (nextState.velocity() - previousState.velocity()) / safeLoopPeriod;

    previousState = nextState;

    return new State(nextState.position(), nextState.velocity(), accelerationRadPerSecSq);
  }

  public State calculate(
      double goalPositionRad,
      double goalVelocityRadPerSec,
      double maxVelocityRadPerSec,
      double maxAccelerationRadPerSecSq,
      double loopPeriodSecs,
      double minPositionRad,
      double maxPositionRad) {
    double safeMinPosition = Math.min(minPositionRad, maxPositionRad);
    double safeMaxPosition = Math.max(minPositionRad, maxPositionRad);
    double constrainedGoalPosition =
        MathUtil.clamp(goalPositionRad, safeMinPosition, safeMaxPosition);

    State nextState =
        calculate(
            constrainedGoalPosition,
            goalVelocityRadPerSec,
            maxVelocityRadPerSec,
            maxAccelerationRadPerSecSq,
            loopPeriodSecs);
    double constrainedPosition =
        MathUtil.clamp(nextState.positionRad(), safeMinPosition, safeMaxPosition);

    if (constrainedPosition != nextState.positionRad()) {
      reset(constrainedPosition, 0.0);
      return new State(constrainedPosition, 0.0, 0.0);
    }

    return nextState;
  }
}
