package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  public double goal = 0.0;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setPID(
        ElevatorConstants.kp,
        ElevatorConstants.Ki,
        ElevatorConstants.Kd,
        ElevatorConstants.Ks,
        ElevatorConstants.Kv,
        ElevatorConstants.Ka,
        ElevatorConstants.Kg,
        ElevatorConstants.acceleration,
        ElevatorConstants.velocity);
  }

  public void runGoal(
      double positionMeters, double velocityMetersPerSec, double acceleration, double torque) {
    io.runPositionSetpoint(positionMeters, velocityMetersPerSec, acceleration, torque);
  }

  public void runMotionMagicPosition(double positionMeters) {
    goal =
        MathUtil.clamp(
            positionMeters,
            ElevatorConstants.elevatorMinimumPosition,
            ElevatorConstants.elevatorMaximumPosition);
    io.runMotionMagicPosition(goal);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void stop() {
    io.stop();
  }

  public double getVelocityMetersPerSec() {
    return (inputs.elevatorLeftVelocityMetersPerSec + inputs.elevatorRightVelocityMetersPerSec) / 2;
  }

  public double getAcceleration() {
    return (inputs.elevatorLeftAcceleration + inputs.elevatorRightAcceleration) / 2;
  }

  public double getPositionMeters() {
    return (inputs.elevatorLeftPositionMeters + inputs.elevatorRightPositionMeters) / 2;
  }

  public double getvolts() {
    return inputs.elevatorLeftAppliedVolts;
  }

  public boolean atGoal() {
    return Math.abs(goal - getPositionMeters()) < ElevatorConstants.elevatorTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    //  SmartDashboard.putNumber("elevator length", inputs.elevatorLeftPositionMeters);
  }
}
