package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Carriage;
import frc.robot.Constants.Climbing;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class ClimbingSubsystem extends SubsystemBase {
  private TalonFX climbtopMotor = new TalonFX(Climbing.climbtopMotorID);
  private TalonFX climbbuttomMotor = new TalonFX(Climbing.climbbuttonMotorID);
  

  private TalonFXConfiguration climbtopMotorConfig = new TalonFXConfiguration();
  private TalonFXConfiguration climbbuttomMotorConfig = new TalonFXConfiguration();

  private MotionMagicVoltage climbingMagic = new MotionMagicVoltage(0);

  private DutyCycleOut voltageout = new DutyCycleOut(0.0);

  public enum climbingState{
    STOP,
    UP,
    DOWM,
    FORWARD,
    BACKWARD
  }

  public climbingState currentclimbState = climbingState.STOP;

  private static ClimbingSubsystem instance;
  public static ClimbingSubsystem getInstance(){
      if(instance == null){
        instance = new ClimbingSubsystem();
      }
      return instance;
    }

  public ClimbingSubsystem() {
    configclimb();
  }

  private void configclimb() {
      /* Motor Inverts and Neutral Mode */
    climbtopMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climbtopMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbtopMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    climbbuttomMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    climbbuttomMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbbuttomMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    /* Current Limiting */
    climbtopMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbtopMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    climbtopMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
    climbtopMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.01;

    climbbuttomMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    climbbuttomMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    climbbuttomMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
    climbbuttomMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.01;

     /* Open and Closed Loop Ramping */
    climbtopMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
    climbtopMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    climbtopMotorConfig.Feedback.SensorToMechanismRatio = Climbing.climbGearRatio;

    climbbuttomMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
    climbbuttomMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    climbbuttomMotorConfig.Feedback.SensorToMechanismRatio = Climbing.climbGearRatio;

    climbtopMotorConfig.Slot0.kP = Climbing.climbKP;
    climbtopMotorConfig.Slot0.kI = Climbing.climbKI;
    climbtopMotorConfig.Slot0.kD = Climbing.climbKD;
    climbtopMotorConfig.Slot0.kV = Climbing.climbKV;
    climbtopMotorConfig.Slot0.kS = Climbing.climbKS;
    climbtopMotorConfig.Slot0.kA = Climbing.climbKA;
    climbtopMotorConfig.MotionMagic.MotionMagicAcceleration = Climbing.Acc;
    climbtopMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Climbing.Vel;
    climbtopMotorConfig.MotionMagic.MotionMagicJerk = Climbing.Jer;

    climbbuttomMotorConfig.Slot0.kP = Climbing.climbKP;
    climbbuttomMotorConfig.Slot0.kI = Climbing.climbKI;
    climbbuttomMotorConfig.Slot0.kD = Climbing.climbKD;
    climbbuttomMotorConfig.Slot0.kV = Climbing.climbKV;
    climbbuttomMotorConfig.Slot0.kS = Climbing.climbKS;
    climbbuttomMotorConfig.Slot0.kA = Climbing.climbKA;
    climbbuttomMotorConfig.MotionMagic.MotionMagicAcceleration = Climbing.Acc;
    climbbuttomMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Climbing.Vel;
    climbbuttomMotorConfig.MotionMagic.MotionMagicJerk = Climbing.Jer;


    climbtopMotor.getConfigurator().apply(climbtopMotorConfig);
    climbbuttomMotor.getConfigurator().apply(climbbuttomMotorConfig);
    restclimbingPos();
  }

  public void setclimbingHeight(double position){
    position = MathUtil.clamp(position, Climbing.MinHeight,Climbing.MaxHeight);
    climbingMagic.Position = position;
    climbtopMotor.setControl(climbingMagic);
    climbbuttomMotor.setControl(new Follower(climbtopMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public void setclimbingVel(double outputpercentage){
    voltageout.Output = outputpercentage;
    climbtopMotor.setControl(voltageout);
    climbbuttomMotor.setControl(new Follower(climbtopMotor.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public boolean atDesireHeight(double height){
    return Math.abs(climbtopMotor.getPosition().getValueAsDouble()-height)<Climbing.climbTolerance;
  }

  public void restclimbingPos(){
    climbtopMotor.getConfigurator().setPosition(0);
    climbbuttomMotor.getConfigurator().setPosition(0);
  }

  public void stopClimbing(){
    climbtopMotor.stopMotor();
    climbbuttomMotor.stopMotor();
  }

  public void setState(climbingState state){
    this.currentclimbState = state;
  }

  @Override
  public void periodic() {
    switch (currentclimbState) {
      case STOP->{
        stopClimbing();
    }
      case FORWARD->{
        setclimbingVel(Climbing.FowardVel);
      }
      case BACKWARD->{
        setclimbingVel(Climbing.BackwardVel);
      }
      case DOWM->{
        setclimbingHeight(Climbing.climbingForPos);
      }
      case UP->{
        setclimbingHeight(Climbing.climbingBackPos);
      }
      default -> {
        
      }
    }
      // SmartDashboard.putNumber("climb Position", climbtopMotor.getPosition().getValueAsDouble());
      // SmartDashboard.putNumber("climb Position", climbbuttomMotor.getPosition().getValueAsDouble());
      // SmartDashboard.putNumber("climb Velocity", climbtopMotor.getVelocity().getValueAsDouble());
      // SmartDashboard.putNumber("climb Velocity", climbbuttomMotor.getVelocity().getValueAsDouble());
      
    }
  }

