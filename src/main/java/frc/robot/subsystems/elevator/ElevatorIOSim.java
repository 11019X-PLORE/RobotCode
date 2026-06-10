package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim leftsim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          ElevatorConstants.elevatorGearRatio,
          ElevatorConstants.jkMetersSquared,
          ElevatorConstants.drumRadiusMeters,
          ElevatorConstants.elevatorMinimumPosition,
          ElevatorConstants.elevatorMaximumPosition,
          false,
          ElevatorConstants.elevatorInitialPosition);

  private final ElevatorSim rightsim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          ElevatorConstants.elevatorGearRatio,
          ElevatorConstants.jkMetersSquared,
          ElevatorConstants.drumRadiusMeters,
          ElevatorConstants.elevatorMinimumPosition,
          ElevatorConstants.elevatorMaximumPosition,
          false,
          ElevatorConstants.elevatorInitialPosition);

  private double appliedVoltage = 0.0;

  private final PIDController controller;

  public ElevatorIOSim() {
    controller = new PIDController(0.0, 0.0, 0.0);
    leftsim.setState(ElevatorConstants.elevatorMaximumPosition / 2, 0);
    rightsim.setState(ElevatorConstants.elevatorMaximumPosition / 2, 0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    leftsim.update(ElevatorConstants.loopPeriodSecs);
    inputs.elevatorLeftPositionMeters = leftsim.getPositionMeters();
    inputs.elevatorLeftVelocityMetersPerSec = leftsim.getVelocityMetersPerSecond();
    inputs.elevatorLeftAppliedVolts = appliedVoltage;
    inputs.elevatorLeftSupplyCurrentAmps = Math.abs(leftsim.getCurrentDrawAmps());

    rightsim.update(ElevatorConstants.loopPeriodSecs);
    inputs.elevatorRightPositionMeters = rightsim.getPositionMeters();
    inputs.elevatorRightVelocityMetersPerSec = rightsim.getVelocityMetersPerSecond();
    inputs.elevatorRightAppliedVolts = appliedVoltage;
    inputs.elevatorRightSupplyCurrentAmps = Math.abs(rightsim.getCurrentDrawAmps());
  }

  @Override
  public void runMotionMagicPosition(double positionMeters) {
    appliedVoltage = controller.calculate(leftsim.getCurrentDrawAmps(), positionMeters);
    appliedVoltage = MathUtil.clamp(appliedVoltage, -12.0, 12.0);
    leftsim.setInputVoltage(appliedVoltage);
    rightsim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    leftsim.setInputVoltage(appliedVoltage);
    rightsim.setInputVoltage(appliedVoltage);
  }
}
