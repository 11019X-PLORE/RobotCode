package frc.robot.util;

import frc.robot.util.Geoffrey.ShooterSetpoint;

public final class SimulationShotParameters {
  public final double velocityMetersPerSec;
  public final double hoodAngleRadians;
  public final double turretYawRadians;

  private SimulationShotParameters(
      double velocityMetersPerSec, double hoodAngleRadians, double turretYawRadians) {
    this.velocityMetersPerSec = velocityMetersPerSec;
    this.hoodAngleRadians = hoodAngleRadians;
    this.turretYawRadians = turretYawRadians;
  }

  public static SimulationShotParameters fromSetpoint(
      ShooterSetpoint setpoint, double lookAheadSeconds, double compensationPercent) {
    double compensationMultiplier = 1.0 + (compensationPercent / 100.0);
    return new SimulationShotParameters(
        (setpoint.shooterVelocityMetersPerSec
                + lookAheadSeconds * setpoint.shooterAccelerationMetersPerSecSquared)
            * compensationMultiplier,
        setpoint.hoodPositionRadians + lookAheadSeconds * setpoint.hoodVelocityRadsPerSec,
        setpoint.turretPositionRadians + lookAheadSeconds * setpoint.turretVelocityRadsPerSec);
  }
}
