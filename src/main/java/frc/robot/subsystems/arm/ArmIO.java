package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Voltage;

public interface ArmIO {
  class ArmIOInputs {
    public boolean armMotorConnected_LF = true;
    public boolean armMotorConnected_LB = true;
    public boolean armMotorConnected_RF = true;
    public boolean armMotorConnected_RB = true;

    public double armPositionRads_LF = 0.0;
    public double armOmegaRadPerSec_LF = 0.0;
    public double armAlphaRadsPerSecSquared_LF = 0.0;
    public double armAppliedVolts_LF = 0.0;
    public double armSupplyCurrentAmps_LF = 0.0;
    public double armTorqueCurrentAmps_LF = 0.0;
    public double armTempCelsius_LF = 0.0;

    public double armPositionRads_LB = 0.0;
    public double armOmegaRadPerSec_LB = 0.0;
    public double armAlphaRadsPerSecSquared_LB = 0.0;
    public double armAppliedVolts_LB = 0.0;
    public double armSupplyCurrentAmps_LB = 0.0;
    public double armTorqueCurrentAmps_LB = 0.0;
    public double armTempCelsius_LB = 0.0;

    public double armPositionRads_RF = 0.0;
    public double armOmegaRadPerSec_RF = 0.0;
    public double armAlphaRadsPerSecSquared_RF = 0.0;
    public double armAppliedVolts_RF = 0.0;
    public double armSupplyCurrentAmps_RF = 0.0;
    public double armTorqueCurrentAmps_RF = 0.0;
    public double armTempCelsius_RF = 0.0;

    public double armPositionRads_RB = 0.0;
    public double armOmegaRadPerSec_RB = 0.0;
    public double armAlphaRadsPerSecSquared_RB = 0.0;
    public double armAppliedVolts_RB = 0.0;
    public double armSupplyCurrentAmps__RB = 0.0;
    public double armTorqueCurrentAmps_RB = 0.0;
    public double armTempCelsius_RB = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Run arm motor at volts */
  default void runVolts(double volts) {}

  /** Run to arm positionsetpoint with feedforward */
  default void runPositionSetpoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {}

  /** Run to arm positionsetpoint with motionmagic */
  default void runMotionMagicArmPosition(
      double angleRads,
      double omegaRadPerSec,
      double alphaRadsPerSecSquared,
      double currentAngle) {}

  /** Run to climb positionsetpoint with motionmagic */
  default void runMotionMagicClimberPosition(double angleRads) {}

  /** Configure arm PID */
  default void setArmPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {}

  /** Configure climb PID */
  default void setElevatorPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {}

  /** select arm or climb Mode */
  default void setPIDMode(boolean enable) {}

  /** resetPosition */
  default void resetPosition(double angleRads) {}

  default void setVoltage(Voltage volt) {}

  default double get() {
    return 0;
  }
  /** Disable output to all motors */
  default void stop() {}

  default void switchBrake(boolean isBrake) {}
  ;
}
