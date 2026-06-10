package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOKrakenFOC implements IntakeIO {
  // hardwares
  private final TalonFX intakeMotor;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<AngularAcceleration> accelerationMetersPerSec2;

  private double wheelRadius = IntakeConstants.wheelRadius;
  private final double reduction = 1.0;

  private final TalonFXConfiguration intakeFXConfiguration = new TalonFXConfiguration();

  // Control
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final MotionMagicVoltage motionmagicControl = new MotionMagicVoltage(0.0);

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public IntakeIOKrakenFOC() {
    intakeMotor = new TalonFX(IntakeConstants.id);
    intakeFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    intakeFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 100.0;
    intakeFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -100.0;
    intakeFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    intakeFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeFXConfiguration.CurrentLimits.StatorCurrentLimit = 120.0;

    intakeFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;
    intakeFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40.0;
    intakeFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 60.0;
    intakeFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.02;

    intakeFXConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    intakeFXConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

    intakeFXConfiguration.Feedback.SensorToMechanismRatio = IntakeConstants.intakeGearRatio;

    intakeFXConfiguration.Slot0.kP = IntakeConstants.Kp;
    intakeFXConfiguration.Slot0.kI = IntakeConstants.Ki;
    intakeFXConfiguration.Slot0.kD = IntakeConstants.Kd;
    intakeFXConfiguration.Slot0.kS = IntakeConstants.Ks;
    intakeFXConfiguration.Slot0.kV = IntakeConstants.Kv;
    intakeFXConfiguration.Slot0.kA = IntakeConstants.Ka;
    // intakeFXConfiguration.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    intakeFXConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        100.0 / IntakeConstants.intakeGearRatio;
    intakeFXConfiguration.MotionMagic.MotionMagicAcceleration =
        intakeFXConfiguration.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    intakeFXConfiguration.MotionMagic.MotionMagicExpo_kV = 0.12 * IntakeConstants.intakeGearRatio;
    intakeFXConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;
    intakeFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

    intakeMotor.getConfigurator().apply(intakeFXConfiguration);

    // Set signals
    position = intakeMotor.getPosition();
    velocity = intakeMotor.getVelocity();
    appliedVoltage = intakeMotor.getMotorVoltage();
    supplyCurrent = intakeMotor.getSupplyCurrent();
    torqueCurrent = intakeMotor.getTorqueCurrent();
    tempCelsius = intakeMotor.getDeviceTemp();
    accelerationMetersPerSec2 = intakeMotor.getAcceleration();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        accelerationMetersPerSec2);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.refreshAll(
                    position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius)
                .isOK());
    inputs.positionRads = Units.rotationsToRadians(position.getValueAsDouble()) / reduction;
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocity.getValueAsDouble()) / reduction;
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.velocityMetersPerSec = inputs.velocityRadsPerSec * wheelRadius;
    inputs.accelerationMetersPerSec2 =
        accelerationMetersPerSec2.getValueAsDouble() * 2 * Math.PI * wheelRadius;
  }

  @Override
  public void runVolts(double volts) {
    intakeMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runTorqueCurrent(double current) {
    intakeMotor.setControl(torqueCurrentFOC.withOutput(current));
  }

  @Override
  public void runVelocitySetpoint(double velocityMetersPerSec, double acceleration) {
    // SmartDashboard.putNumber("run", velocityMetersPerSec);
    intakeMotor.setControl(
        velocityTorqueCurrentFOC
            .withVelocity(velocityMetersPerSec / ((2 * Math.PI) * wheelRadius))
            .withAcceleration(acceleration / (2 * Math.PI)));
  }

  @Override
  public void runMotionMagicPosition(double angleRads) {
    motionmagicControl.Position = angleRads / (2 * Math.PI);
    intakeMotor.setControl(motionmagicControl);
  }

  @Override
  public void stop() {
    intakeMotor.setControl(neutralOut);
  }
}
