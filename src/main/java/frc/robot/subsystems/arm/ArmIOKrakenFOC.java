package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

public class ArmIOKrakenFOC implements ArmIO {
  // hardwares
  private final TalonFX armLeftForwardMotor;
  private final TalonFX armLeftBackMotor;
  private final TalonFX armRightForwardMotor;
  private final TalonFX armRightBackMotor;

  // Status Signals
  private final StatusSignal<Angle> armPosition_LF;
  private final StatusSignal<AngularVelocity> armVelocity_LF;
  private final StatusSignal<Voltage> armAppliedVolts_LF;
  private final StatusSignal<Current> armSupplyCurrent_LF;
  private final StatusSignal<Current> armTorqueCurrent_LF;
  private final StatusSignal<AngularAcceleration> armacceleration_LF;
  private final StatusSignal<Temperature> armTempCelsius_LF;

  private final StatusSignal<Angle> armPosition_LB;
  private final StatusSignal<AngularVelocity> armVelocity_LB;
  private final StatusSignal<Voltage> armAppliedVolts_LB;
  private final StatusSignal<Current> armSupplyCurrent_LB;
  private final StatusSignal<Current> armTorqueCurrent_LB;
  private final StatusSignal<AngularAcceleration> armacceleration_LB;
  private final StatusSignal<Temperature> armTempCelsius_LB;

  private final StatusSignal<Angle> armPosition_RF;
  private final StatusSignal<AngularVelocity> armVelocity_RF;
  private final StatusSignal<Voltage> armAppliedVolts_RF;
  private final StatusSignal<Current> armSupplyCurrent_RF;
  private final StatusSignal<Current> armTorqueCurrent_RF;
  private final StatusSignal<AngularAcceleration> armacceleration_RF;
  private final StatusSignal<Temperature> armTempCelsius_RF;

  private final StatusSignal<Angle> armPosition_RB;
  private final StatusSignal<AngularVelocity> armVelocity_RB;
  private final StatusSignal<Voltage> armAppliedVolts_RB;
  private final StatusSignal<Current> armSupplyCurrent_RB;
  private final StatusSignal<Current> armTorqueCurrent_RB;
  private final StatusSignal<AngularAcceleration> armacceleration_RB;
  private final StatusSignal<Temperature> armTempCelsius_RB;

  private double armRadius = ArmConstants.armRadius;

  private final TalonFXConfiguration armFXConfiguration = new TalonFXConfiguration();

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
  private final MotionMagicVoltage motionmagicControl = new MotionMagicVoltage(0.0);
  private final NeutralOut neutralControl = new NeutralOut();

  public ArmIOKrakenFOC() {
    armLeftForwardMotor = new TalonFX(ArmConstants.armLeftForwardMotorid);
    armLeftBackMotor = new TalonFX(ArmConstants.armLeftBackMotorid);
    armRightForwardMotor = new TalonFX(ArmConstants.armRightForwardMotorid);
    armRightBackMotor = new TalonFX(ArmConstants.armRightBackMotorid);

    armFXConfiguration.Feedback.SensorToMechanismRatio = ArmConstants.armGearRatio;

    armFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.01;
    // armFXConfiguration.MotorOutput.DutyCycleNeutralDeadband = 0.008;

    /*Torque Current Limiting */
    armFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 100.0;
    armFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -100.0;
    armFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

    armFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    armFXConfiguration.CurrentLimits.StatorCurrentLimit = 120.0;

    /* Current Limiting */
    // armFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // armFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
    // armFXConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 60;
    // armFXConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.01;

    /* Open and Closed Loop Ramping */
    armFXConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
    armFXConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

    armLeftForwardMotor.getConfigurator().apply(armFXConfiguration);
    armLeftBackMotor.getConfigurator().apply(armFXConfiguration);
    armRightForwardMotor.getConfigurator().apply(armFXConfiguration);
    armRightBackMotor.getConfigurator().apply(armFXConfiguration);

    //    armLeftBackMotor.setControl(new Follower(armLeftForwardMotor.getDeviceID(), false));
    //    armRightBackMotor.setControl(new Follower(armRightForwardMotor.getDeviceID(), false));

    resetPosition(ArmConstants.armInitialAngle);

    // Set signals
    armPosition_LF = armLeftForwardMotor.getPosition();
    armVelocity_LF = armLeftForwardMotor.getVelocity();
    armAppliedVolts_LF = armLeftForwardMotor.getMotorVoltage();
    armSupplyCurrent_LF = armLeftForwardMotor.getSupplyCurrent();
    armTorqueCurrent_LF = armLeftForwardMotor.getTorqueCurrent();
    armacceleration_LF = armLeftForwardMotor.getAcceleration();
    armTempCelsius_LF = armLeftForwardMotor.getDeviceTemp();

    armPosition_LB = armLeftBackMotor.getPosition();
    armVelocity_LB = armLeftBackMotor.getVelocity();
    armAppliedVolts_LB = armLeftBackMotor.getMotorVoltage();
    armSupplyCurrent_LB = armLeftBackMotor.getSupplyCurrent();
    armTorqueCurrent_LB = armLeftBackMotor.getTorqueCurrent();
    armacceleration_LB = armLeftBackMotor.getAcceleration();
    armTempCelsius_LB = armLeftBackMotor.getDeviceTemp();

    armPosition_RF = armRightForwardMotor.getPosition();
    armVelocity_RF = armRightForwardMotor.getVelocity();
    armAppliedVolts_RF = armRightForwardMotor.getMotorVoltage();
    armSupplyCurrent_RF = armRightForwardMotor.getSupplyCurrent();
    armTorqueCurrent_RF = armRightForwardMotor.getTorqueCurrent();
    armacceleration_RF = armRightForwardMotor.getAcceleration();
    armTempCelsius_RF = armRightForwardMotor.getDeviceTemp();

    armPosition_RB = armRightBackMotor.getPosition();
    armVelocity_RB = armRightBackMotor.getVelocity();
    armAppliedVolts_RB = armRightBackMotor.getMotorVoltage();
    armSupplyCurrent_RB = armRightBackMotor.getSupplyCurrent();
    armTorqueCurrent_RB = armRightBackMotor.getTorqueCurrent();
    armacceleration_RB = armRightBackMotor.getAcceleration();
    armTempCelsius_RB = armRightBackMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        armPosition_LF,
        armVelocity_LF,
        armAppliedVolts_LF,
        armSupplyCurrent_LF,
        armTorqueCurrent_LF,
        armacceleration_LF,
        armTempCelsius_LF,
        armPosition_LB,
        armVelocity_LB,
        armAppliedVolts_LB,
        armSupplyCurrent_LB,
        armTorqueCurrent_LB,
        armacceleration_LB,
        armTempCelsius_LB,
        armPosition_RF,
        armVelocity_RF,
        armAppliedVolts_RF,
        armSupplyCurrent_RF,
        armTorqueCurrent_RF,
        armacceleration_RF,
        armTempCelsius_RF,
        armPosition_RB,
        armVelocity_RB,
        armAppliedVolts_RB,
        armSupplyCurrent_RB,
        armTorqueCurrent_RB,
        armacceleration_RB,
        armTempCelsius_RB);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armMotorConnected_LF =
        BaseStatusSignal.refreshAll(
                armPosition_LF,
                armVelocity_LF,
                armAppliedVolts_LF,
                armSupplyCurrent_LF,
                armTorqueCurrent_LF,
                armacceleration_LF,
                armTempCelsius_LF)
            .isOK();

    inputs.armMotorConnected_LB =
        BaseStatusSignal.refreshAll(
                armPosition_LB,
                armVelocity_LB,
                armAppliedVolts_LB,
                armSupplyCurrent_LB,
                armTorqueCurrent_LB,
                armacceleration_LB,
                armTempCelsius_LB)
            .isOK();

    inputs.armMotorConnected_RF =
        BaseStatusSignal.refreshAll(
                armPosition_RF,
                armVelocity_RF,
                armAppliedVolts_RF,
                armSupplyCurrent_RF,
                armTorqueCurrent_RF,
                armacceleration_RF,
                armTempCelsius_RF)
            .isOK();

    inputs.armMotorConnected_RB =
        BaseStatusSignal.refreshAll(
                armPosition_RB,
                armVelocity_RB,
                armAppliedVolts_RB,
                armSupplyCurrent_RB,
                armTorqueCurrent_RB,
                armacceleration_RB,
                armTempCelsius_RB)
            .isOK();

    inputs.armPositionRads_LF = armPosition_LF.getValueAsDouble() * 2 * Math.PI;
    inputs.armOmegaRadPerSec_LF = armVelocity_LF.getValueAsDouble() * 2 * Math.PI;
    inputs.armAppliedVolts_LF = armAppliedVolts_LF.getValueAsDouble();
    inputs.armSupplyCurrentAmps_LF = armSupplyCurrent_LF.getValueAsDouble();
    inputs.armTorqueCurrentAmps_LF = armTorqueCurrent_LF.getValueAsDouble();
    inputs.armTempCelsius_LF = armTempCelsius_LF.getValueAsDouble();
    inputs.armAlphaRadsPerSecSquared_LF = armacceleration_LF.getValueAsDouble() * 2 * Math.PI;

    inputs.armPositionRads_LB = armPosition_LB.getValueAsDouble() * 2 * Math.PI;
    inputs.armOmegaRadPerSec_LB = armVelocity_LB.getValueAsDouble() * 2 * Math.PI;
    inputs.armAppliedVolts_LB = armAppliedVolts_LB.getValueAsDouble();
    inputs.armSupplyCurrentAmps_LB = armSupplyCurrent_LB.getValueAsDouble();
    inputs.armTorqueCurrentAmps_LB = armTorqueCurrent_LB.getValueAsDouble();
    inputs.armTempCelsius_LB = armTempCelsius_LB.getValueAsDouble();
    inputs.armAlphaRadsPerSecSquared_LB = armacceleration_LB.getValueAsDouble() * 2 * Math.PI;

    inputs.armPositionRads_RF = armPosition_RF.getValueAsDouble() * 2 * Math.PI;
    inputs.armOmegaRadPerSec_RF = armVelocity_RF.getValueAsDouble() * 2 * Math.PI;
    inputs.armAppliedVolts_RF = armAppliedVolts_RF.getValueAsDouble();
    inputs.armSupplyCurrentAmps_RF = armSupplyCurrent_RF.getValueAsDouble();
    inputs.armTorqueCurrentAmps_RF = armTorqueCurrent_RF.getValueAsDouble();
    inputs.armTempCelsius_RF = armTempCelsius_RF.getValueAsDouble();
    inputs.armAlphaRadsPerSecSquared_RF = armacceleration_RF.getValueAsDouble() * 2 * Math.PI;
  }

  @Override
  public void runVolts(double volts) {
    armLeftForwardMotor.setControl(voltageControl.withOutput(volts));
    armLeftBackMotor.setControl(voltageControl.withOutput(volts));
    armRightForwardMotor.setControl(voltageControl.withOutput(-volts));
    armRightBackMotor.setControl(voltageControl.withOutput(-volts));
    // armLeftBackMotor.setControl(new Follower(ArmConstants.armLeftForwardMotorid, false));
    // armRightForwardMotor.setControl(voltageControl.withOutput(-volts));
    // armRightBackMotor.setControl(new Follower(ArmConstants.armRightForwardMotorid, false));
    // armRightForwardMotor.setControl(new Follower(ArmConstants.armLeftForwardMotorid, true));
    // armRightBackMotor.setControl(new Follower(ArmConstants.armLeftForwardMotorid, true));
  }

  @Override
  public void runPositionSetpoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    // SmartDashboard.putNumber("torquefinal",torque);
    // SmartDashboard.putString("running mode","PositionSetpoint");
    armLeftForwardMotor.setControl(
        positionControl
            .withPosition(angleRads / (2 * Math.PI))
            .withVelocity(omegaRadPerSec / (2 * Math.PI))
            .withFeedForward(torque));
    armLeftBackMotor.setControl(
        new Follower(armLeftForwardMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    armRightForwardMotor.setControl(
        new Follower(armLeftForwardMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    armRightBackMotor.setControl(
        new Follower(armLeftForwardMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void runMotionMagicArmPosition(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    // SmartDashboard.putString("running mode","motionmagic");
    motionmagicControl.Position = angleRads / (2 * Math.PI);
    // armLeftForwardMotor.setControl(motionmagicControl.withFeedForward(torque));
    armLeftForwardMotor.setControl(
        motionmagicControl
            .withPosition(angleRads / (2 * Math.PI))
            .withSlot(0)
            .withFeedForward(torque));
    armLeftBackMotor.setControl(
        new Follower(armLeftForwardMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    armRightForwardMotor.setControl(
        new Follower(armLeftForwardMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    armRightBackMotor.setControl(
        new Follower(armLeftForwardMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void runMotionMagicClimberPosition(double angleRads) {
    motionmagicControl.Position = angleRads / (2 * Math.PI);
    armLeftForwardMotor.setControl(
        motionmagicControl
            .withPosition(angleRads / (2 * Math.PI))
            .withSlot(0)
            .withFeedForward(null));
    armLeftBackMotor.setControl(
        new Follower(armLeftForwardMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    armRightForwardMotor.setControl(
        new Follower(armLeftForwardMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    armRightBackMotor.setControl(
        new Follower(armLeftForwardMotor.getDeviceID(), MotorAlignmentValue.Opposed));
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
    armFXConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armFXConfiguration.Slot0.kP = kP;
    armFXConfiguration.Slot0.kI = kI;
    armFXConfiguration.Slot0.kD = kD;
    armFXConfiguration.Slot0.kS = kS;
    armFXConfiguration.Slot0.kV = kV;
    armFXConfiguration.Slot0.kA = kA;
    armFXConfiguration
        .MotionMagic
        .withMotionMagicCruiseVelocity(1) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(2) // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));
    // armFXConfiguration.MotionMagic.MotionMagicAcceleration=MotionMagicAcceleration;
    // armFXConfiguration.MotionMagic.MotionMagicCruiseVelocity=MotionMagicCruiseVelocity;
    // armFXConfiguration.MotionMagic.MotionMagicJerk=2;
    armLeftForwardMotor.getConfigurator().apply(armFXConfiguration, 0.01);
    armLeftBackMotor.getConfigurator().apply(armFXConfiguration, 0.01);
    armRightForwardMotor.getConfigurator().apply(armFXConfiguration, 0.01);
    armRightBackMotor.getConfigurator().apply(armFXConfiguration, 0.01);
  }

  @Override
  public void setElevatorPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {
    armFXConfiguration.Slot0.kP = kP;
    armFXConfiguration.Slot0.kI = kI;
    armFXConfiguration.Slot0.kD = kD;
    armFXConfiguration.Slot0.kS = kS;
    armFXConfiguration.Slot0.kV = kV;
    armFXConfiguration.Slot0.kA = kA;
    armFXConfiguration.MotionMagic.MotionMagicAcceleration = MotionMagicAcceleration;
    armFXConfiguration.MotionMagic.MotionMagicCruiseVelocity = MotionMagicCruiseVelocity;
    armLeftForwardMotor.getConfigurator().apply(armFXConfiguration, 0.01);
    armLeftBackMotor.getConfigurator().apply(armFXConfiguration, 0.01);
    armRightForwardMotor.getConfigurator().apply(armFXConfiguration, 0.01);
    armRightBackMotor.getConfigurator().apply(armFXConfiguration, 0.01);
  }

  @Override
  public void resetPosition(double angleRads) {
    angleRads = angleRads / (2 * Math.PI);
    armLeftForwardMotor.getConfigurator().setPosition(angleRads);
    armLeftBackMotor.getConfigurator().setPosition(angleRads);
    armRightForwardMotor.getConfigurator().setPosition(angleRads);
    armRightBackMotor.getConfigurator().setPosition(angleRads);
  }

  @Override
  public void setPIDMode(boolean enable) {
    if (enable) {
      setArmPID(
          ArmConstants.kp,
          ArmConstants.Ki,
          ArmConstants.Kd,
          ArmConstants.Ks,
          ArmConstants.Kv,
          ArmConstants.Ka,
          ArmConstants.Kg,
          ArmConstants.acceleration,
          ArmConstants.velocity);
    } else {
      setElevatorPID(
          ArmConstants.kp_Climber,
          ArmConstants.Ki_Climber,
          ArmConstants.Kd_Climber,
          ArmConstants.Ks_Climber,
          ArmConstants.Kv_Climber,
          ArmConstants.Ka_Climber,
          ArmConstants.acceleration_Climber,
          ArmConstants.velocity_Climber);
    }
  }

  @Override
  public void stop() {
    armLeftForwardMotor.setControl(neutralControl);
    armLeftBackMotor.setControl(neutralControl);
    armRightForwardMotor.setControl(neutralControl);
    armRightBackMotor.setControl(neutralControl);
  }

  @Override
  public double get() {
    return armLeftForwardMotor.get();
  }

  @Override
  public void setVoltage(Voltage volt) {
    runVolts(volt.in(edu.wpi.first.units.BaseUnits.VoltageUnit));
  }

  @Override
  public void switchBrake(boolean isBrake) {
    if (isBrake) {
      armFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      armFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }
    armLeftForwardMotor.getConfigurator().apply(armFXConfiguration, 0.01);
    armLeftBackMotor.getConfigurator().apply(armFXConfiguration, 0.01);
    armRightForwardMotor.getConfigurator().apply(armFXConfiguration, 0.01);
    armRightBackMotor.getConfigurator().apply(armFXConfiguration, 0.01);
  }
}
