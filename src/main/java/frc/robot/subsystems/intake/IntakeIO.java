package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double accelerationMetersPerSec2 = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Run intake motor at volts */
  default void runVolts(double volts) {}

  /** Run to intake velocity setpoint with feedforward */
  default void runVelocitySetpoint(double velocityMetersPerSec, double acceleration) {}

  default void runMotionMagicPosition(double angleRads) {}

  default void runTorqueCurrent(double current) {}

  /** Disable output to all motors */
  default void stop() {}
}
