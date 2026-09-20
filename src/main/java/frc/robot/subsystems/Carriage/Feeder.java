package frc.robot.subsystems.Carriage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants.Carriage;
import frc.robot.Constants.Intake;

public class Feeder extends SubsystemBase {
  private TalonFX carriageFeederMotor  = new TalonFX(Carriage.carriageFeederMotorID);
  private TalonFXConfiguration carriageFeederMotorconfig = new TalonFXConfiguration();
  private VelocityVoltage carriageVelDutycycle = new VelocityVoltage(0);

  private TalonFX shooterFeederMotor  = new TalonFX(Carriage.shooterFeederMotorID);
  private TalonFXConfiguration shooterFeederMotorconfig = new TalonFXConfiguration();
  private VelocityVoltage shooterFeederMotorVelDutycycle = new VelocityVoltage(0);

  private final DigitalInput carriagePhotoSensor = new DigitalInput(Carriage.elevatorSensorChannel);
  private DigitalInput intakePhotoSensor= new DigitalInput(Intake.photoSensorChannel);

  public Feeder() {
    configfeeder();
  }

  private void configfeeder(){
   /* Motor Inverts and Neutral Mode */
   carriageFeederMotorconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
   carriageFeederMotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
   carriageFeederMotorconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

   carriageFeederMotorconfig.Feedback.SensorToMechanismRatio=Carriage.carriageFeederGearRatio;

   /* Current Limiting */
   carriageFeederMotorconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
   carriageFeederMotorconfig.CurrentLimits.SupplyCurrentLimit = 40;
   carriageFeederMotorconfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
   carriageFeederMotorconfig.CurrentLimits.SupplyCurrentLowerTime = 0.02;

    /* Open and Closed Loop Ramping */
   carriageFeederMotorconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
   carriageFeederMotorconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

   carriageFeederMotorconfig.Slot0.kP = Carriage.carriage_KP;
   carriageFeederMotorconfig.Slot0.kI = Carriage.carriage_KI;
   carriageFeederMotorconfig.Slot0.kD = Carriage.carriage_KD;
   carriageFeederMotorconfig.Slot0.kV = Carriage.carriage_KV;
   carriageFeederMotorconfig.Slot0.kS = Carriage.carriage_KS;
   carriageFeederMotorconfig.Slot0.kA = Carriage.carriage_KA;

    carriageFeederMotor.getConfigurator().apply(carriageFeederMotorconfig);
   /* Motor Inverts and Neutral Mode */
   shooterFeederMotorconfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
   shooterFeederMotorconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
   shooterFeederMotorconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

   shooterFeederMotorconfig.Feedback.SensorToMechanismRatio=Carriage.shooterFeederGearRatio;

   /* Current Limiting */
   shooterFeederMotorconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
   shooterFeederMotorconfig.CurrentLimits.SupplyCurrentLimit = 40;
   shooterFeederMotorconfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
   shooterFeederMotorconfig.CurrentLimits.SupplyCurrentLowerTime = 0.02;

    /* Open and Closed Loop Ramping */
   shooterFeederMotorconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
   shooterFeederMotorconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

   shooterFeederMotorconfig.Slot0.kP = Carriage.shooter_KP;
   shooterFeederMotorconfig.Slot0.kI = Carriage.shooter_KI;
   shooterFeederMotorconfig.Slot0.kD = Carriage.shooter_KD;
   shooterFeederMotorconfig.Slot0.kV = Carriage.shooter_KV;
   shooterFeederMotorconfig.Slot0.kS = Carriage.shooter_KS;
   shooterFeederMotorconfig.Slot0.kA = Carriage.shooter_KA;

   shooterFeederMotor.getConfigurator().apply(shooterFeederMotorconfig);
  }

  private void carriageFeederControl(double velocity){
    carriageVelDutycycle.Velocity = velocity;
    carriageFeederMotor.setControl(carriageVelDutycycle);
  }

  public void setCarriageFeederVelocity(double velocity){
    carriageFeederControl(Conversions.MPSToRPS(velocity,Carriage.carriageRollerDiameter*Math.PI));
  }

  private void shooterFeederControl(double velocity){
    shooterFeederMotorVelDutycycle.Velocity = velocity;
    shooterFeederMotor.setControl(shooterFeederMotorVelDutycycle);
  }

  public void setShooterFeederVelocity(double velocity){
    shooterFeederControl(Conversions.MPSToRPS(velocity,Carriage.feederRollerDiameter*Math.PI));
  }

  public void stopCarriage(){
    carriageFeederMotor.stopMotor();
    shooterFeederMotor.stopMotor();
  }


  public double getCarriageFeederVelocity(){
    return Conversions.RPSToMPS(carriageFeederMotor.getVelocity().getValueAsDouble(),Carriage.carriageRollerDiameter*Math.PI);
   
  }

  public double getshooterFeederVelocity(){
    return Conversions.RPSToMPS(shooterFeederMotor.getVelocity().getValueAsDouble(),Carriage.carriageRollerDiameter*Math.PI);
  }

  public boolean sensorDetected(){
    return !carriagePhotoSensor.get();
  }
  public boolean sensorintakeDetected(){
    return !intakePhotoSensor.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("getCarriageFeederVelocity", getCarriageFeederVelocity());
    // SmartDashboard.putNumber("getshooterFeederVelocity", getshooterFeederVelocity());
  }
}
