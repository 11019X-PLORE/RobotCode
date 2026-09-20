package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Carriage;
import frc.robot.Constants.Shooter;

public class Triggershooting extends SubsystemBase {
  private TalonFX triggerMotor  = new TalonFX(Shooter.triggerMotorID);
  private TalonFXConfiguration triggerMotorconfig = new TalonFXConfiguration();
  private VelocityVoltage triggerMotorVelDutycycle = new VelocityVoltage(0);
  private final DigitalInput triggerPhotoSensor = new DigitalInput(Shooter.photoSensorChannel);

  public enum triggerState{
    STOP,
    FORWARD,
    BACKWARD,
    SHOOTING
  }
  public triggerState currenttriggerState = triggerState.STOP;
  public Triggershooting() {
    configTrigger();
  }

  private void configTrigger(){
    /* Motor Inverts and Neutral Mode */
   triggerMotorconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
   triggerMotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
   triggerMotorconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

   triggerMotorconfig.Feedback.SensorToMechanismRatio=Constants.Shooter.triggerGearRatio;

   /* Current Limiting */
   triggerMotorconfig.CurrentLimits.SupplyCurrentLimitEnable = false;
   triggerMotorconfig.CurrentLimits.SupplyCurrentLimit = 20;
   triggerMotorconfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
   triggerMotorconfig.CurrentLimits.SupplyCurrentLowerTime = 0.02;

    /* Open and Closed Loop Ramping */
   triggerMotorconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
   triggerMotorconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;

   triggerMotorconfig.Slot0.kP = Shooter.trrigerKP;
   triggerMotorconfig.Slot0.kI = Shooter.trrigerKI;
   triggerMotorconfig.Slot0.kD = Shooter.trrigerKD;
   triggerMotorconfig.Slot0.kV = Shooter.trrigerKV;

   triggerMotorconfig.Voltage.PeakForwardVoltage = 12;
   triggerMotorconfig.Voltage.PeakReverseVoltage = -12;

   triggerMotor.getConfigurator().apply(triggerMotorconfig);
  }

  private void triggerControl(double velocity){
    triggerMotorVelDutycycle.Velocity = velocity;
    triggerMotor.setControl(triggerMotorVelDutycycle);
  }

  public void settriggerVelocity(double velocity){
    triggerControl(Conversions.MPSToRPS(velocity,Shooter.triggerRollerDiameter*Math.PI));
  }

  public void stopTrigger(){
    triggerMotor.stopMotor();
  }

  public boolean sensorDetected(){
    return !triggerPhotoSensor.get();
  }

  public double getTriggerVelocity(){
    return Conversions.RPSToMPS(triggerMotor.getVelocity().getValueAsDouble(),Shooter.triggerRollerDiameter*Math.PI);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("triggerVelocity", getTriggerVelocity());
  }
}
