package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOKrakenFOC implements ElevatorIO {
  // hardwares
  private final TalonFX elevatorLeftMotor;
  private final TalonFX elevatorRightMotor;

  // Status Signals
  private final StatusSignal<Angle> elevatorLeftPosition;
  private final StatusSignal<AngularVelocity> elevatorLeftVelocity;
  private final StatusSignal<Voltage> elevatorLeftAppliedVolts;
  private final StatusSignal<Current> elevatorLeftSupplyCurrent;
  private final StatusSignal<Current> elevatorLeftTorqueCurrent;
  private final StatusSignal<AngularAcceleration> elevatorLeftacceleration;
  private final StatusSignal<Temperature> elevatorLeftTempCelsius;

  private final StatusSignal<Angle> elevatorRightPosition;
  private final StatusSignal<AngularVelocity> elevatorRightVelocity;
  private final StatusSignal<Voltage> elevatorRightAppliedVolts;
  private final StatusSignal<Current> elevatorRightSupplyCurrent;
  private final StatusSignal<Current> elevatorRightTorqueCurrent;
  private final StatusSignal<AngularAcceleration> elevatorRightacceleration;
  private final StatusSignal<Temperature> elevatorRightTempCelsius;

  private double elevatorRadius = ElevatorConstants.elevatorRadius;

  private final TalonFXConfiguration elevatorFXConfiguration = new TalonFXConfiguration();

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC motionmagicControl =
      new MotionMagicTorqueCurrentFOC(0.0);
  private final NeutralOut neutralControl = new NeutralOut();

  public ElevatorIOKrakenFOC() {
    elevatorLeftMotor = new TalonFX(ElevatorConstants.elevatorLeftId);
    elevatorRightMotor = new TalonFX(ElevatorConstants.elevatorRightId);
    elevatorFXConfiguration.Feedback.SensorToMechanismRatio = ElevatorConstants.elevatorGearRatio;

    elevatorFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /*Torque Current Limiting */
    elevatorFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 100.0;
    elevatorFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -100.0;
    elevatorFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    elevatorFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorFXConfiguration.CurrentLimits.StatorCurrentLimit = 120.0;

    /* Current Limiting */
    elevatorFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
    elevatorFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 60;
    elevatorFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.01;

    /* Open and Closed Loop Ramping */
    elevatorFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    elevatorFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

    elevatorLeftMotor.getConfigurator().apply(elevatorFXConfiguration);
    elevatorRightMotor.getConfigurator().apply(elevatorFXConfiguration);
    resetPosition(ElevatorConstants.elevatorInitialPosition);

    // Set signals
    elevatorLeftPosition = elevatorLeftMotor.getPosition();
    elevatorLeftVelocity = elevatorLeftMotor.getVelocity();
    elevatorLeftAppliedVolts = elevatorLeftMotor.getMotorVoltage();
    elevatorLeftSupplyCurrent = elevatorLeftMotor.getSupplyCurrent();
    elevatorLeftTorqueCurrent = elevatorLeftMotor.getTorqueCurrent();
    elevatorLeftacceleration = elevatorLeftMotor.getAcceleration();
    elevatorLeftTempCelsius = elevatorLeftMotor.getDeviceTemp();

    elevatorRightPosition = elevatorRightMotor.getPosition();
    elevatorRightVelocity = elevatorRightMotor.getVelocity();
    elevatorRightAppliedVolts = elevatorRightMotor.getMotorVoltage();
    elevatorRightSupplyCurrent = elevatorRightMotor.getSupplyCurrent();
    elevatorRightTorqueCurrent = elevatorRightMotor.getTorqueCurrent();
    elevatorRightacceleration = elevatorRightMotor.getAcceleration();
    elevatorRightTempCelsius = elevatorRightMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        elevatorLeftPosition,
        elevatorLeftVelocity,
        elevatorLeftAppliedVolts,
        elevatorLeftSupplyCurrent,
        elevatorLeftTorqueCurrent,
        elevatorLeftacceleration,
        elevatorLeftTempCelsius,
        elevatorRightPosition,
        elevatorRightVelocity,
        elevatorRightAppliedVolts,
        elevatorRightSupplyCurrent,
        elevatorRightTorqueCurrent,
        elevatorRightacceleration,
        elevatorRightTempCelsius);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorLeftMotorConnected =
        BaseStatusSignal.refreshAll(
                elevatorLeftPosition,
                elevatorLeftVelocity,
                elevatorLeftAppliedVolts,
                elevatorLeftSupplyCurrent,
                elevatorLeftTorqueCurrent,
                elevatorLeftacceleration,
                elevatorLeftTempCelsius)
            .isOK();
    inputs.elevatorRightMotorConnected =
        BaseStatusSignal.refreshAll(
                elevatorRightPosition,
                elevatorRightVelocity,
                elevatorRightAppliedVolts,
                elevatorRightSupplyCurrent,
                elevatorRightTorqueCurrent,
                elevatorRightacceleration,
                elevatorRightTempCelsius)
            .isOK();
    inputs.elevatorLeftPositionMeters =
        elevatorLeftPosition.getValueAsDouble() * 2 * Math.PI * elevatorRadius;
    inputs.elevatorLeftVelocityMetersPerSec =
        elevatorLeftVelocity.getValueAsDouble() * 2 * Math.PI * elevatorRadius;
    inputs.elevatorLeftAppliedVolts = elevatorLeftAppliedVolts.getValueAsDouble();
    inputs.elevatorLeftSupplyCurrentAmps = elevatorLeftSupplyCurrent.getValueAsDouble();
    inputs.elevatorLeftTorqueCurrentAmps = elevatorLeftTorqueCurrent.getValueAsDouble();
    inputs.elevatorLeftTempCelsius = elevatorLeftTempCelsius.getValueAsDouble();
    inputs.elevatorLeftAcceleration =
        elevatorLeftacceleration.getValueAsDouble() * 2 * Math.PI * elevatorRadius;

    inputs.elevatorRightPositionMeters =
        elevatorRightPosition.getValueAsDouble() * 2 * Math.PI * elevatorRadius;
    inputs.elevatorRightVelocityMetersPerSec =
        elevatorRightVelocity.getValueAsDouble() * 2 * Math.PI * elevatorRadius;
    inputs.elevatorRightAppliedVolts = elevatorRightAppliedVolts.getValueAsDouble();
    inputs.elevatorRightSupplyCurrentAmps = elevatorRightSupplyCurrent.getValueAsDouble();
    inputs.elevatorRightTorqueCurrentAmps = elevatorRightTorqueCurrent.getValueAsDouble();
    inputs.elevatorRightTempCelsius = elevatorRightTempCelsius.getValueAsDouble();
    inputs.elevatorRightAcceleration =
        elevatorRightacceleration.getValueAsDouble() * 2 * Math.PI * elevatorRadius;
  }

  @Override
  public void runVolts(double volts) {
    elevatorLeftMotor.setControl(voltageControl.withOutput(volts));
    elevatorRightMotor.setControl(
        new Follower(elevatorLeftMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void runPositionSetpoint(
      double positionMeters, double velocityMetersPerSec, double acceleration, double torque) {
    elevatorLeftMotor.setControl(
        positionControl
            .withPosition(positionMeters / (2 * Math.PI * elevatorRadius))
            .withVelocity(velocityMetersPerSec / (2 * Math.PI * elevatorRadius))
            .withFeedForward(acceleration * ElevatorConstants.Ka + ElevatorConstants.Kt * torque));
    elevatorRightMotor.setControl(
        new Follower(elevatorLeftMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void runMotionMagicPosition(double positionMeters) {
    elevatorLeftMotor.setControl(
        motionmagicControl.withPosition(positionMeters).withFeedForward(ElevatorConstants.Kg));
    elevatorRightMotor.setControl(
        new Follower(elevatorLeftMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  @Override
  public void setPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double KG,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {
    elevatorFXConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevatorFXConfiguration.Slot0.kP = kP;
    elevatorFXConfiguration.Slot0.kI = kI;
    elevatorFXConfiguration.Slot0.kD = kD;
    elevatorFXConfiguration.Slot0.kS = kS;
    elevatorFXConfiguration.Slot0.kV = kV;
    elevatorFXConfiguration.Slot0.kA = kA;
    elevatorFXConfiguration.Slot0.kG = KG;
    elevatorFXConfiguration.MotionMagic.MotionMagicAcceleration = MotionMagicAcceleration;
    elevatorFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity;
    elevatorLeftMotor.getConfigurator().apply(elevatorFXConfiguration, 0.01);
    elevatorRightMotor.getConfigurator().apply(elevatorFXConfiguration, 0.01);
  }

  @Override
  public void resetPosition(double positionMeters) {
    positionMeters = positionMeters / (2 * Math.PI);
    elevatorLeftMotor.getConfigurator().setPosition(positionMeters);
    elevatorRightMotor.getConfigurator().setPosition(positionMeters);
  }

  @Override
  public void stop() {
    elevatorLeftMotor.setControl(neutralControl);
    elevatorRightMotor.setControl(neutralControl);
  }
}
