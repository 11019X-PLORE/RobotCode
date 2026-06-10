package frc.robot.subsystems.elevator;

public interface ElevatorIO {
  class ElevatorIOInputs {
    public boolean elevatorLeftMotorConnected = true;
    public boolean elevatorRightMotorConnected = true;

    public double elevatorLeftPositionMeters = 0.0;
    public double elevatorLeftVelocityMetersPerSec = 0.0;
    public double elevatorLeftAcceleration = 0.0;
    public double elevatorLeftAppliedVolts = 0.0;
    public double elevatorLeftSupplyCurrentAmps = 0.0;
    public double elevatorLeftTorqueCurrentAmps = 0.0;
    public double elevatorLeftTempCelsius = 0.0;

    public double elevatorRightPositionMeters = 0.0;
    public double elevatorRightVelocityMetersPerSec = 0.0;
    public double elevatorRightAcceleration = 0.0;
    public double elevatorRightAppliedVolts = 0.0;
    public double elevatorRightSupplyCurrentAmps = 0.0;
    public double elevatorRightTorqueCurrentAmps = 0.0;
    public double elevatorRightTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run elevator motor at volts */
  default void runVolts(double volts) {}

  /** Run to elevator positionsetpoint with feedforward */
  default void runPositionSetpoint(
      double positionMeters, double velocityMetersPerSec, double acceleration, double torque) {}

  /** Run to elevator positionsetpoint with motionmagic */
  default void runMotionMagicPosition(double positionMeters) {}

  /** Configure elevator PID */
  default void setPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {}

  /** resetPosition */
  default void resetPosition(double angleRads) {}

  /** Disable output to all motors */
  default void stop() {}
}
