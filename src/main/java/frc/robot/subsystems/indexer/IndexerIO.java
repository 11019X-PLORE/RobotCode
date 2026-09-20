package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;

    public boolean rotatorconnected;
    public double rotatorvelocityRadsPerSec;
    public double rotatorappliedVoltage;
    public double rotatorsupplyCurrentAmps;
    public double rotatortorqueCurrentAmps;
    public double rotatortempCelsius;
  }

  public static enum IndexerIOOutputMode {
    COAST,
    VELOCITY,
  }

  public static class IndexerIOOutputs {
    public IndexerIOOutputMode mode = IndexerIOOutputMode.VELOCITY;
    public double velocityRadsPerSec = 0.0;

    // rotator outputs (merged from rotatorIO)
    public IndexerIOOutputMode rotatorMode = IndexerIOOutputMode.VELOCITY;
    public double rotatorVelocityRadsPerSec = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void applyOutputs(IndexerIOOutputs outputs) {}

  /** Configure indexer PID */
  public default void setPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  /** Configure rotator PID */
  public default void setrotatorPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
