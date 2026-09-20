
package frc.robot.subsystems.Carriage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Carriage;
import frc.robot.Constants.Climbing;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Climber.ClimbingSubsystem;
import frc.robot.subsystems.Climber.ClimbingSubsystem.climbingState;
import frc.robot.subsystems.Intake.IntakeSystem;
import frc.robot.subsystems.Intake.IntakeSystem.intakeState;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.Shooter.Triggershooting;
import frc.robot.subsystems.Shooter.Turret;

public class CarriageSystem extends SubsystemBase {
  Feeder m_feeder;
  Elevator m_elevator;
  LEDSubsystem m_LED;
  ClimbingSubsystem m_climb;
  Turret m_turret;
  Triggershooting m_triggershooting;
  

  public enum carriageState{
    STOP,
    INTAKE,
    SPIT,
    AMPSCORING,
    TRAPSCORING,
    SHOOTING,
    Test_UP,
    Test_DOWN,
    TO_AMP,
    TELEOP_ELEVATOR,
  }
  carriageState currentCarriageState=carriageState.STOP;

  public CarriageSystem(Feeder feeder, Elevator elevator, LEDSubsystem LED,ClimbingSubsystem climb,Turret turret,Triggershooting triggershooting) {
    this.m_feeder = feeder;
    this.m_elevator = elevator;
    this.m_LED = LED;
    this.m_climb = climb;
    this.m_turret = turret;
    this.m_triggershooting = triggershooting;
  }

  public void setState(carriageState state){
    currentCarriageState=state;
  }

  @Override
  public void periodic() {
    switch (currentCarriageState) {
      case STOP->{
        m_feeder.stopCarriage();
        m_elevator.setelevatorHeight(Carriage.elevatorGroundPosition);
    }
      case INTAKE->{
        m_feeder.setCarriageFeederVelocity(Carriage.intakeVel_CarriageFeeder);
      }
      case SPIT->{
        m_feeder.setCarriageFeederVelocity(Carriage.spitVel_CarriageFeeder);
        m_feeder.setShooterFeederVelocity(Carriage.spitVel_ShootingFeeder);
      }
      case SHOOTING->{
        m_feeder.setCarriageFeederVelocity(Carriage.shootingVel_CarriageFeeder);
        m_feeder.setShooterFeederVelocity(Carriage.shootingVel_ShootingFeeder);
      }
      case TO_AMP->{
        if(m_feeder.sensorDetected()&&m_turret.atDesireAngle()){
          m_feeder.stopCarriage();
          m_triggershooting.stopTrigger();
        }else{
          m_feeder.setCarriageFeederVelocity(Carriage.amp_scoring_CarriageShooterVel);
          m_feeder.setShooterFeederVelocity(Carriage.amp_scoring_ShootingFeederVel);
          m_triggershooting.settriggerVelocity(Shooter.ampScoreVel);
        }
        m_LED.update(LEDState.TO_AMP);
      }
      case AMPSCORING->{
        if(m_turret.atDesireAngle()){
          m_triggershooting.settriggerVelocity(Shooter.ampScoreVel);
          m_feeder.setShooterFeederVelocity(Carriage.amp_scoring_ShootingFeederVel);
        }else{
          m_triggershooting.stopTrigger();
          m_feeder.stopCarriage();
        }
        if(m_feeder.sensorDetected()){
            m_elevator.setelevatorHeight(Carriage.elevatorAmpPosition);
              if(m_elevator.atDesireHeight(Carriage.elevatorAmpPosition)){
                m_feeder.setCarriageFeederVelocity(Carriage.amp_scoring_CarriageShooterVel);
              }else{
                m_feeder.stopCarriage();
              }
        }else{
            m_feeder.setCarriageFeederVelocity(Carriage.amp_scoring_CarriageShooterVel);
          }
            m_LED.update(LEDState.TO_AMP);
        }
      case TRAPSCORING->{
        if(m_feeder.sensorDetected()){
          m_elevator.setelevatorHeight(Carriage.elevatorTrapPosition);
          if(m_elevator.atDesireHeight(Carriage.elevatorTrapPosition)
          &&m_climb.currentclimbState==climbingState.UP
          &&m_climb.atDesireHeight(Climbing.climbingBackPos)){
            m_feeder.setCarriageFeederVelocity(Carriage.amp_scoring_CarriageShooterVel);
          }else{
            m_feeder.stopCarriage();
          }
        }
        m_LED.update(LEDState.Climb);
      }
      case TELEOP_ELEVATOR->{
        m_elevator.setelevatorHeight(Carriage.elevatorTrapPosition);
      }
      case Test_UP->{
        m_elevator.setElevatorDutycycleOut(0.2);
      }
      case Test_DOWN->{
        m_elevator.setElevatorDutycycleOut(-0.2);
      }
      default -> {
        
      }
    }
  }
}
