package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MotorOutputLimiter;

public class IndexerIOReal implements IndexerIO {
  // === Indexer Motor ===
  private final TalonFX indexTalon;

  private final TalonFXConfiguration indexConfig;

  private final StatusSignal<Angle> indexerPosition;
  private final StatusSignal<AngularVelocity> indexerVelocity;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerSupplyCurrent;
  private final StatusSignal<Current> indexerTorqueCurrent;
  private final StatusSignal<Temperature> indexerTemp;

  private final VelocityTorqueCurrentFOC indexerVelocityControl = new VelocityTorqueCurrentFOC(0.0);
  private final NeutralOut indexerCoastControl = new NeutralOut();

  // === rotator Motors (merged from rotatorIOReal) ===
  private final TalonFX rotatorTalonFX;
  private final TalonFXConfiguration rotatorConfig;

  private final StatusSignal<AngularVelocity> rotatorVelocity;
  private final StatusSignal<Voltage> rotatorAppliedVolts;
  private final StatusSignal<Current> rotatorSupplyCurrent;
  private final StatusSignal<Current> rotatorTorqueCurrent;
  private final StatusSignal<Temperature> rotatorTemp;

  private final VelocityTorqueCurrentFOC rotatorVelocityControl = new VelocityTorqueCurrentFOC(0.0);
  private final NeutralOut rotatorCoastControl = new NeutralOut();

  public IndexerIOReal() {
    // === Configure Indexer Motor ===
    indexTalon = new TalonFX(IndexerConstants.kIndexerId);
    indexConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        IndexerConstants.kIndexInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IndexerConstants.kIndexerGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(
                        Amps.of(
                            MotorOutputLimiter.scaleCurrent(
                                IndexerConstants.kIndexerStatorCurrentLimit)))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(MotorOutputLimiter.scaleCurrent(40)))
                    .withSupplyCurrentLimitEnable(false)
                    .withSupplyCurrentLowerLimit(Amps.of(MotorOutputLimiter.scaleCurrent(60)))
                    .withSupplyCurrentLowerTime(0.1)
                    .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> indexTalon.getConfigurator().apply(indexConfig));

    indexerPosition = indexTalon.getPosition();
    indexerVelocity = indexTalon.getVelocity();
    indexerAppliedVolts = indexTalon.getMotorVoltage();
    indexerSupplyCurrent = indexTalon.getSupplyCurrent();
    indexerTorqueCurrent = indexTalon.getTorqueCurrent();
    indexerTemp = indexTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerPosition,
        indexerVelocity,
        indexerAppliedVolts,
        indexerSupplyCurrent,
        indexerTorqueCurrent);

    // === Configure rotator Motors ===
    rotatorTalonFX = new TalonFX(IndexerConstants.kRotatorId);

    rotatorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        IndexerConstants.kRotatorInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(IndexerConstants.kRotatorGearRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(MotorOutputLimiter.scaleCurrent(100.0)))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(MotorOutputLimiter.scaleCurrent(40)))
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLowerLimit(Amps.of(MotorOutputLimiter.scaleCurrent(60)))
                    .withSupplyCurrentLowerTime(0.1)
                    .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> rotatorTalonFX.getConfigurator().apply(rotatorConfig));
    rotatorVelocity = rotatorTalonFX.getVelocity();
    rotatorAppliedVolts = rotatorTalonFX.getMotorVoltage();
    rotatorSupplyCurrent = rotatorTalonFX.getSupplyCurrent();
    rotatorTorqueCurrent = rotatorTalonFX.getTorqueCurrent();
    rotatorTemp = rotatorTalonFX.getDeviceTemp();
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rotatorVelocity,
        rotatorAppliedVolts,
        rotatorSupplyCurrent,
        rotatorTorqueCurrent,
        rotatorTemp);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                indexerPosition,
                indexerVelocity,
                indexerAppliedVolts,
                indexerSupplyCurrent,
                indexerTorqueCurrent,
                indexerTemp)
            .isOK();

    inputs.positionRads = Units.rotationsToRadians(indexerPosition.getValueAsDouble());
    inputs.velocityRadsPerSec = Units.rotationsToRadians(indexerVelocity.getValueAsDouble());
    inputs.appliedVoltage = indexerAppliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = indexerSupplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = indexerTorqueCurrent.getValueAsDouble();
    inputs.tempCelsius = indexerTemp.getValueAsDouble();

    inputs.rotatorconnected =
        BaseStatusSignal.refreshAll(
                rotatorVelocity,
                rotatorAppliedVolts,
                rotatorSupplyCurrent,
                rotatorTorqueCurrent,
                rotatorTemp)
            .isOK();
    inputs.rotatorvelocityRadsPerSec = Units.rotationsToRadians(rotatorVelocity.getValueAsDouble());
    inputs.rotatorappliedVoltage = rotatorAppliedVolts.getValueAsDouble();
    inputs.rotatorsupplyCurrentAmps = rotatorSupplyCurrent.getValueAsDouble();
    inputs.rotatortorqueCurrentAmps = rotatorTorqueCurrent.getValueAsDouble();
    inputs.rotatortempCelsius = rotatorTemp.getValueAsDouble();
  }

  @Override
  public void setPID(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP;
    cfg.kI = kI;
    cfg.kD = kD;
    cfg.kS = kS;
    cfg.kV = kV;
    cfg.kA = kA;
    tryUntilOk(5, () -> indexTalon.getConfigurator().apply(cfg));
  }

  @Override
  public void setrotatorPID(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    Slot0Configs cfg = new Slot0Configs();
    cfg.kP = kP;
    cfg.kI = kI;
    cfg.kD = kD;
    cfg.kS = kS;
    cfg.kV = kV;
    cfg.kA = kA;
    tryUntilOk(5, () -> rotatorTalonFX.getConfigurator().apply(cfg));
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {

    // === Apply Indexer Outputs ===
    switch (outputs.mode) {
      case COAST -> {
        indexTalon.setControl(indexerCoastControl);
      }
      case VELOCITY -> {
        indexTalon.setControl(
            indexerVelocityControl.withVelocity(
                Units.radiansToRotations(outputs.velocityRadsPerSec)));
      }
    }

    switch (outputs.rotatorMode) {
      case COAST -> {
        rotatorTalonFX.setControl(rotatorCoastControl);
      }
      case VELOCITY -> {
        rotatorTalonFX.setControl(
            rotatorVelocityControl.withVelocity(
                Units.radiansToRotations(outputs.rotatorVelocityRadsPerSec)));
      }
    }
  }
}
