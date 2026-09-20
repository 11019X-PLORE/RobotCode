package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.MotorOutputLimiter;

public class IndexerIOSim implements IndexerIO {
  // === Indexer sim ===
  private static final DCMotor indexerMotorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim indexerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              indexerMotorModel, .025, IndexerConstants.kIndexerGearRatio),
          indexerMotorModel);

  private PIDController indexerController = new PIDController(0.5, 0, 0, Constants.loopPeriodSecs);
  private double indexerCurrentOutput = 0.0;
  private double indexerAppliedVolts = 0.0;

  // === rotator sim ===
  private static final DCMotor rotatorMotorModel = DCMotor.getKrakenX60(1);
  private static final DCMotorSim rotatorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              rotatorMotorModel, .025, IndexerConstants.kRotatorGearRatio),
          rotatorMotorModel);

  private PIDController rotatorController = new PIDController(0.5, 0, 0, Constants.loopPeriodSecs);
  private double rotatorCurrentOutput = 0.0;
  private double rotatorAppliedVolts = 0.0;

  public IndexerIOSim() {}

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Indexer sim update
    double indexerOutputAsVolt =
        indexerMotorModel.getVoltage(
            indexerCurrentOutput, indexerSim.getAngularVelocityRadPerSec());
    indexerAppliedVolts = MotorOutputLimiter.scaleVoltage(indexerOutputAsVolt);
    indexerSim.setInputVoltage(MathUtil.clamp(indexerAppliedVolts, -12.0, 12.0));
    indexerSim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRads = indexerSim.getAngularPositionRad();
    inputs.velocityRadsPerSec = indexerSim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = indexerAppliedVolts;
    inputs.supplyCurrentAmps = indexerSim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = indexerCurrentOutput;
    inputs.tempCelsius = 0.0;
    // rotator sim update
    double rotatorOutputAsVolt =
        rotatorMotorModel.getVoltage(
            rotatorCurrentOutput, rotatorSim.getAngularVelocityRadPerSec());
    rotatorAppliedVolts = MotorOutputLimiter.scaleVoltage(rotatorOutputAsVolt);
    rotatorSim.setInputVoltage(MathUtil.clamp(rotatorAppliedVolts, -12.0, 12.0));
    rotatorSim.update(Constants.loopPeriodSecs);

    inputs.rotatorconnected = true;
    inputs.rotatorvelocityRadsPerSec = rotatorSim.getAngularVelocityRadPerSec();
    inputs.rotatorappliedVoltage = rotatorAppliedVolts;
    inputs.rotatorsupplyCurrentAmps = rotatorSim.getCurrentDrawAmps();
    inputs.rotatortorqueCurrentAmps = rotatorCurrentOutput;
    inputs.rotatortempCelsius = 0.0;
  }

  // 同步添加缺少的 Indexer PID 设置方法
  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    indexerController.setPID(kP, kI, kD);
  }

  // 同步添加缺少的 rotator PID 设置方法
  @Override
  public void setrotatorPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    rotatorController.setPID(kP, kI, kD);
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    // === Indexer ===
    switch (outputs.mode) {
      case COAST -> {
        indexerCurrentOutput = 0.0;
      }
      case VELOCITY -> {
        indexerCurrentOutput =
            MotorOutputLimiter.scaleCurrent(
                indexerController.calculate(
                    indexerSim.getAngularVelocityRadPerSec(), outputs.velocityRadsPerSec));
      }
    }

    // === rotator ===
    switch (outputs.rotatorMode) {
      case COAST -> {
        rotatorCurrentOutput = 0.0;
      }
      case VELOCITY -> {
        rotatorCurrentOutput =
            MotorOutputLimiter.scaleCurrent(
                rotatorController.calculate(
                    rotatorSim.getAngularVelocityRadPerSec(), outputs.rotatorVelocityRadsPerSec));
      }
    }
  }
}
