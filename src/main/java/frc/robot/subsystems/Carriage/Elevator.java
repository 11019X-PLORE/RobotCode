package frc.robot.subsystems.Carriage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Carriage;

public class Elevator extends SubsystemBase {
  private TalonFX elevatorMotor = new TalonFX(Carriage.elevatorMotorID);
  private TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
  // private MotionMagicVoltage elevatorMagic = new MotionMagicVoltage(0, true, 0.0, 0, false, false, false);
  private MotionMagicVoltage elevatorMagic = new MotionMagicVoltage(0);
  public Elevator() {
    configelevator();
  }

  private void configelevator(){
    /* Motor Inverts and Neutral Mode */
    elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    elevatorMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
    elevatorMotorConfig.CurrentLimits.SupplyCurrentLowerTime= 0.01;


    /* Open and Closed Loop Ramping */
    elevatorMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
    elevatorMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    elevatorMotorConfig.Feedback.SensorToMechanismRatio = Carriage.elevatorGearRatio;

    elevatorMotorConfig.Slot0.kP = Carriage.ElevatorKP;
    elevatorMotorConfig.Slot0.kI = Carriage.ElevatorKI;
    elevatorMotorConfig.Slot0.kD = Carriage.ElevatorKD;
    elevatorMotorConfig.Slot0.kV = Carriage.ElevatorKV;
    elevatorMotorConfig.Slot0.kS = Carriage.ElevatorKS;
    elevatorMotorConfig.Slot0.kA = Carriage.ElevatorKA;
    elevatorMotorConfig.Slot0.kG = Carriage.ElevatorKG;
    elevatorMotorConfig.MotionMagic.MotionMagicAcceleration = Carriage.Acc;
    elevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Carriage.Vel;
    elevatorMotorConfig.MotionMagic.MotionMagicJerk = Carriage.Jer;

    elevatorMotor.getConfigurator().apply(elevatorMotorConfig);
    resetPosistion();
  }

  public void setelevatorHeight(double position){
    position =MathUtil.clamp(position, Carriage.MinHeight,Carriage.MaxHeight);
    elevatorMagic.Position = position;
    elevatorMotor.setControl(elevatorMagic);
  }

   public void setElevatorDutycycleOut(double outputpercentage){
    DutyCycleOut voltageout = new DutyCycleOut(outputpercentage);
    elevatorMotor.setControl(voltageout);
  }

  public void stopElevator(){
    elevatorMotor.stopMotor();
  }

  private void resetPosistion(){
    elevatorMotor.getConfigurator().setPosition(0);
  }

  public double getHeight(){
    // return elevatorMotor.getPosition().getValue();
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public boolean atDesireHeight(double height){
    return Math.abs(getHeight()-height)<Carriage.elevatortolerance;
  }

  @Override
  public void periodic() {
    //  SmartDashboard.putNumber("ElevatorHeight",getHeight());
  }
}
