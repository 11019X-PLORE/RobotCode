package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim sim;
  private final DCMotor gearbox;
  private double appliedVoltage = 0.0;
  private double reduction = 1.0;
  private double moi = 0.01;

  public IntakeIOSim() {
    gearbox = DCMotor.getKrakenX60(1);
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, moi, reduction), gearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    inputs.connected = true;
    sim.update(0.02);
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps =
        gearbox.getCurrent(sim.getAngularVelocityRadPerSec(), appliedVoltage);
  }

  @Override
  public void runTorqueCurrent(double current) {
    runVolts(gearbox.getVoltage(gearbox.getTorque(current), sim.getAngularVelocityRadPerSec()));
  }

  @Override
  public void runMotionMagicPosition(double current) {
    runVolts(gearbox.getVoltage(gearbox.getTorque(current), sim.getAngularVelocityRadPerSec()));
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
