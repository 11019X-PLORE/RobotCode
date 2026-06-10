package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private static final double autoStartAngle = ArmConstants.autoStartAngle;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(2),
          ArmConstants.armGearRatio,
          ArmConstants.jkMetersSquared,
          ArmConstants.armLength,
          ArmConstants.armMinimumAngle,
          ArmConstants.armMaximumAngle,
          false,
          Units.degreesToRadians(0.0));

  private final ProfiledPIDController controller;
  private double appliedVoltage = 0.0;
  private double positionOffset = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean closedLoop = true;
  private boolean wasNotAuto = true;

  public ArmIOSim() {
    controller = new ProfiledPIDController(0.0, 0.0, 0.0, new Constraints(50.0, 100.0));
    sim.setState(0.0, 0.0);
    setPosition(0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // Reset at start of auto
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    sim.update(ArmConstants.loopPeriodSecs);

    inputs.armPositionRads_LF = sim.getAngleRads() + positionOffset;
    inputs.armOmegaRadPerSec_LF = sim.getVelocityRadPerSec();
    inputs.armAppliedVolts_LF = appliedVoltage;
    inputs.armSupplyCurrentAmps_LF = sim.getCurrentDrawAmps();
    inputs.armTorqueCurrentAmps_LF = sim.getCurrentDrawAmps();
    inputs.armTempCelsius_LF = 0.0;

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void runMotionMagicArmPosition(
      double angleRads, double feedforward, double currentElevatorPosition, double torque) {
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    if (controllerNeedsReset) {
      controller.reset(0.0);
      ;
      controllerNeedsReset = false;
    }
    runVolts(controller.calculate(sim.getAngleRads(), angleRads + positionOffset) + feedforward);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setArmPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {
    controller.setPID(kP, kI, kD);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }

  private void setPosition(double position) {
    positionOffset = position - sim.getAngleRads();
  }
}
