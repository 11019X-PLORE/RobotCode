package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;

  private final IntakeIOInputs intakeInputs = new IntakeIOInputs();
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final Alert intakemotorDisconnectedAlert;

  private final Timer timer = new Timer();
  private int exceedCount = 0;
  private double monitorDuration = 2.0;
  private boolean getTorqueCurrent;

  public Intake(IntakeIO io) {
    this.intakeIO = io;
    intakemotorDisconnectedAlert =
        new Alert(" intakemotor disconnected!", Alert.AlertType.kWarning);
  }

  public enum IntakeState {
    FORWAR,
    BACKWARD,
    STOP,
  }

  IntakeState intakeState = IntakeState.STOP;

  public void runGoal(double velocityRadsPerSec, double acceleration) {
    intakeIO.runVelocitySetpoint(velocityRadsPerSec, acceleration);
  }

  public void runVolts(double volts) {
    intakeIO.runVolts(volts);
  }

  /** Stops motors. */
  public void stop() {
    intakeIO.stop();
  }

  /** Get velocity of rolley in m/s. */
  public double getVelocityMetersPerSec() {
    return intakeInputs.velocityMetersPerSec;
  }

  public double getvolts() {
    return intakeInputs.appliedVoltage;
  }

  public double getAcceleration() {
    return intakeInputs.accelerationMetersPerSec2;
  }

  public double getTorqueCurrentAmps() {
    return intakeInputs.supplyCurrentAmps;
  }

  public boolean isStall() { // TODO, not working, won't stop
    double velo = getVelocityMetersPerSec();
    if (Math.abs(getTorqueCurrentAmps()) >= IntakeConstants.TorqueCurrentAmps
        && Math.abs(velo) <= IntakeConstants.VelocityMax
        && Math.abs(getAcceleration()) <= IntakeConstants.AccelerationMax) {
      return true;
    } else {
      // motor not running
      return false;
    }
  }

  public void runMotionMagicPosition(double angleRads) {
    intakeIO.runMotionMagicPosition(angleRads);
  }

  public void setState(IntakeState state) {
    intakeState = state;
  }
  /*
   //SysIdRoutine start
   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
   private final MutVoltage m_appliedVoltage = Volts.mutable(0);
   // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
   private final MutAngle m_distance = Radian.mutable(0);
   // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
   private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

   // Create a new SysId routine for characterizing the drive.
   private final SysIdRoutine m_sysIdRoutine =
       new SysIdRoutine(
           // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
           new SysIdRoutine.Config(),
           new SysIdRoutine.Mechanism(
               // Tell SysId how to plumb the driving voltage to the motors.
               voltage -> {
                 setVoltage(voltage);
               },
               // Tell SysId how to record a frame of data for each motor on the mechanism being
               // characterized.
               log -> {
                 // Record a frame for the left motors.  Since these share an encoder, we consider
                 // the entire group to be one motor.
                 log.motor("arm-all")
                     .voltage(
                         m_appliedVoltage.mut_replace(
                             get() * RobotController.getBatteryVoltage(), Volts))
                     .angularPosition(m_distance.mut_replace(getAngleRads()-Math.toRadians(20), Radians))
                     .angularVelocity(
                         m_velocity.mut_replace(getOmegaRadPerSec(), RadiansPerSecond));
               },
               // Tell SysId to make generated commands require this subsystem, suffix test state in
               // WPILog with this subsystem's name ("drive")
               this));

     public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
         return m_sysIdRoutine.quasistatic(direction);
     }

     public Command sysIdDynamic(SysIdRoutine.Direction direction) {
         return m_sysIdRoutine.dynamic(direction);
     }
     //SysIdRoutine end
  */

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    intakemotorDisconnectedAlert.set(!inputs.connected);
    switch (intakeState) {
      case FORWAR -> {
        runVolts(IntakeConstants.scoreForwardVolts);
      }
      case BACKWARD -> {
        runVolts(IntakeConstants.intakeVolts);
      }
      case STOP -> {
        stop();
      }
    }
    // SmartDashboard.putString("intakeState", intakeState.toString());
    Logger.recordOutput("Intake/State", intakeState.toString());
    // Logger.processInputs("intake", intakeInputs);
    // Logger.processInputs("Sensor", sensorInputs);
  }
}
