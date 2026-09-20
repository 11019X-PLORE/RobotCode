// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Carriage;
import frc.robot.Constants.Climbing;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Carriage.CarriageSystem;
import frc.robot.subsystems.Carriage.CarriageSystem.carriageState;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.Shooter.ShooterSystem;
import frc.robot.subsystems.Shooter.Triggershooting;
import frc.robot.subsystems.Shooter.Turret;

public class IntakeSystem extends SubsystemBase {
  private TalonFX intakeMotor  = new TalonFX(Constants.Intake.intakeMotorID);
  // private TalonFX climbtopMotor = new TalonFX(Climbing.climbtopMotorID);
  private TalonFXConfiguration intakeMotorconfig = new TalonFXConfiguration();
  private VelocityVoltage intakeVelDutycycle = new VelocityVoltage(0);
  public enum intakeState{
    INTAKE,
    SHOOTING,
    SPIT,
    AUTOINTAKE,
    STOP,
  }
  intakeState currentIntakeState=intakeState.STOP;

  
  LEDSubsystem m_LED;
  CarriageSystem m_carriage;
  Triggershooting m_trigger;
  Turret m_turret;
  private static final double PERIOD = 0.005;  // 100Hz (10ms period)
  /** Creates a new Intake. */
  public IntakeSystem(LEDSubsystem LED,CarriageSystem carriage,Triggershooting trigger,Turret turret) {
    this.m_LED=LED;
    this.m_carriage=carriage;
    this.m_trigger=trigger;
    this.m_turret=turret;
    configintakes();
  }
  private void configintakes(){
    intakeMotorconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeMotorconfig.MotorOutput.NeutralMode = Constants.Intake.intakeNeutralMode;
    intakeMotorconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    intakeMotorconfig.Feedback.SensorToMechanismRatio=Constants.Intake.intakeGearRatio;

    /* Current Limiting */
    intakeMotorconfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeMotorconfig.CurrentLimits.SupplyCurrentLimit = 20;
    intakeMotorconfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
    intakeMotorconfig.CurrentLimits.SupplyCurrentLowerTime = 0.02;

     /* Open and Closed Loop Ramping */
    intakeMotorconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 2;
    intakeMotorconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    intakeMotorconfig.Slot0.kP = Constants.Intake.KP;
    intakeMotorconfig.Slot0.kI = Constants.Intake.KI;
    intakeMotorconfig.Slot0.kD = Constants.Intake.KD;
    intakeMotorconfig.Slot0.kV = Constants.Intake.KV;
    intakeMotorconfig.Slot0.kS = Constants.Intake.KS;
    intakeMotorconfig.Slot0.kA = Constants.Intake.KA;

    intakeMotor.getConfigurator().apply(intakeMotorconfig);    
  }
  

  private void intakeControl(double velocity){
    intakeVelDutycycle.Velocity = velocity;
    intakeMotor.setControl(intakeVelDutycycle);
  }
  public void setIntakeingVelocity(double velocity){
    intakeControl(Conversions.MPSToRPS(velocity, Constants.Intake.intakeRollerDiameter*Math.PI));
  }
  public void stopIntake(){
    intakeMotor.stopMotor();
  }
  public void setState(intakeState state){
    currentIntakeState=state;
  }
  
  // public double getIntakeVelocity(){
  //   return Conversions.RPSToMPS(intakeMotor.getVelocity().getValue(),Constants.Intake.intakeRollerDiameter*Math.PI);
  // }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber( "intakespeed",intakeMotor.getVelocity().getValue());
    switch (currentIntakeState) {
      case STOP->{
        stopIntake();
        if(!m_trigger.sensorDetected()){
          m_LED.update(LEDState.UNLOADED);
        }
      }
      case SHOOTING->{
        setIntakeingVelocity(Constants.Intake.shootingVelocity);
      }
      case INTAKE->{
        if(m_trigger.sensorDetected()){
          intakeMotor.stopMotor();
          m_carriage.setState(carriageState.STOP);
          m_trigger.stopTrigger();
          m_LED.update(LEDState.LOADED);
        }else{
          setIntakeingVelocity(Constants.Intake.intakeVelocity);
          m_carriage.setState(carriageState.SHOOTING);
          m_trigger.settriggerVelocity(Shooter.intakeVel);
          m_LED.update(LEDState.UNLOADED);
        }
      }
      case SPIT->{
        setIntakeingVelocity(Constants.Intake.ejectingVelocity);
        m_carriage.setState(carriageState.SPIT);
        m_trigger.settriggerVelocity(Shooter.spitVel);
      }
      default->{

      }
    }
  }

}
