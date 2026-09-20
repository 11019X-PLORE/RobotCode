package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputMode;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOOutputs;
import frc.robot.util.FullSubsystem;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends FullSubsystem {
  // --- Indexer Tunable Numbers ---
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Indexer/kI");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD");
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Indexer/kS");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Indexer/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Indexer/kA");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Indexer/kG");

  private static final LoggedTunableNumber kIntakeVelocity =
      new LoggedTunableNumber("Indexer/kIntakeVelocity");
  private static final LoggedTunableNumber kOuttakeVelocity =
      new LoggedTunableNumber("Indexer/kOuttakeVelocity");
  private static final LoggedTunableNumber kActiveVelocity =
      new LoggedTunableNumber("Indexer/kActiveVelocity");
  private static final LoggedTunableNumber kShootVelocity =
      new LoggedTunableNumber("Indexer/kShootVelocity");
  private static final LoggedTunableNumber velocityTolerance =
      new LoggedTunableNumber("Indexer/VelocityTolerance"); // rad/s
  private static final LoggedTunableNumber atGoalDebounce =
      new LoggedTunableNumber("Indexer/AtGoalDebounce", 0.2);

  // --- Rotator Tunable Numbers ---
  private static final LoggedTunableNumber kRotator_kP =
      new LoggedTunableNumber("Indexer/Rotator_kP");
  private static final LoggedTunableNumber kRotator_kI =
      new LoggedTunableNumber("Indexer/Rotator_kI");
  private static final LoggedTunableNumber kRotator_kD =
      new LoggedTunableNumber("Indexer/Rotator_kD");
  private static final LoggedTunableNumber kRotator_kS =
      new LoggedTunableNumber("Indexer/Rotator_kS");
  private static final LoggedTunableNumber kRotator_kV =
      new LoggedTunableNumber("Indexer/Rotator_kV");
  private static final LoggedTunableNumber kRotator_kA =
      new LoggedTunableNumber("Indexer/Rotator_kA");
  private static final LoggedTunableNumber kRotator_kG =
      new LoggedTunableNumber("Indexer/Rotator_kG");

  private static final LoggedTunableNumber kRotatorIntakeVelocity =
      new LoggedTunableNumber("Indexer/RotatorIntakeVelocity");
  private static final LoggedTunableNumber kRotatorOuttakeVelocity =
      new LoggedTunableNumber("Indexer/RotatorOuttakeVelocity");
  private static final LoggedTunableNumber kRotatorShootVelocity =
      new LoggedTunableNumber("Indexer/RotatorShootVelocity");
  private static final LoggedTunableNumber kRotatorMaxAcceleration =
      new LoggedTunableNumber("Indexer/RotatorMaxAccelerationRadPerSecSq");
  private static final LoggedTunableNumber kRotatorStallVelocityThreshold =
      new LoggedTunableNumber("Indexer/RotatorStallVelocityThreshold");
  private static final LoggedTunableNumber kRotatorStallTorqueCurrentThreshold =
      new LoggedTunableNumber("Indexer/RotatorStallTorqueCurrentThreshold");

  static {
    if (Robot.isSimulation()) {
      kP.initDefault(0.035);
      kI.initDefault(0.0);
      kD.initDefault(0.1);
      kS.initDefault(0.0);
      kV.initDefault(0.0);
      kA.initDefault(0.0);
      kG.initDefault(0.0);
      velocityTolerance.initDefault(10.0);
      kIntakeVelocity.initDefault(100.0);
      kOuttakeVelocity.initDefault(-100.0);
      kActiveVelocity.initDefault(100.0);
      kShootVelocity.initDefault(100.0);

      kRotator_kP.initDefault(0.035);
      kRotator_kI.initDefault(0.0);
      kRotator_kD.initDefault(0.0);
      kRotator_kS.initDefault(0.0);
      kRotator_kV.initDefault(0.0);
      kRotator_kA.initDefault(0.0);
      kRotator_kG.initDefault(0.0);
      kRotatorIntakeVelocity.initDefault(150.0);
      kRotatorOuttakeVelocity.initDefault(IndexerConstants.kRotatorOuttakeVelocity);
      kRotatorShootVelocity.initDefault(150.0);
      kRotatorMaxAcceleration.initDefault(IndexerConstants.kRotatorMaxAccelerationRadPerSecSq);
      kRotatorStallVelocityThreshold.initDefault(IndexerConstants.kRotatorStallVelocityThreshold);
      kRotatorStallTorqueCurrentThreshold.initDefault(
          IndexerConstants.kRotatorStallTorqueCurrentThreshold);
    } else {
      kP.initDefault(IndexerConstants.kP);
      kI.initDefault(IndexerConstants.kI);
      kD.initDefault(IndexerConstants.kD);
      kS.initDefault(IndexerConstants.kS);
      kV.initDefault(IndexerConstants.kV);
      kA.initDefault(IndexerConstants.kA);
      kG.initDefault(IndexerConstants.kG);
      velocityTolerance.initDefault(IndexerConstants.kVelocityTolerance);
      kIntakeVelocity.initDefault(IndexerConstants.kIntakeVelocity);
      kOuttakeVelocity.initDefault(IndexerConstants.kOuttakeVelocity);
      kActiveVelocity.initDefault(IndexerConstants.kActiveVelocity);
      kShootVelocity.initDefault(IndexerConstants.kShootVelocity);

      kRotator_kP.initDefault(IndexerConstants.kRotator_kP);
      kRotator_kI.initDefault(0.0);
      kRotator_kD.initDefault(0.0);
      kRotator_kS.initDefault(0.0);
      kRotator_kV.initDefault(0.0);
      kRotator_kA.initDefault(0.0);
      kRotator_kG.initDefault(0.0);
      kRotatorIntakeVelocity.initDefault(IndexerConstants.kRotatorIntakeVelocity);
      kRotatorOuttakeVelocity.initDefault(IndexerConstants.kRotatorOuttakeVelocity);
      kRotatorShootVelocity.initDefault(IndexerConstants.kRotatorShootVelocity);
      kRotatorMaxAcceleration.initDefault(IndexerConstants.kRotatorMaxAccelerationRadPerSecSq);
      kRotatorStallVelocityThreshold.initDefault(IndexerConstants.kRotatorStallVelocityThreshold);
      kRotatorStallTorqueCurrentThreshold.initDefault(
          IndexerConstants.kRotatorStallTorqueCurrentThreshold);
    }
  }

  // --- IO & Inputs ---
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final IndexerIOOutputs outputs = new IndexerIOOutputs();

  public enum IndexerGoal {
    INTAKE,
    SHOOT,
    FORCE_SHOOT,
    OUTTAKE,
    STOP
  }

  @Getter @Setter @AutoLogOutput private IndexerGoal goal = IndexerGoal.STOP;

  private enum RotatorRecoveryState {
    NORMAL,
    RECOVERING,
    LOCKED_OUT
  }

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean atGoal = false;

  private boolean atGoalRotator = false;

  // --- Hardware Safety & Alerts ---
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer rotatorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer rotatorStallDebouncer =
      new Debouncer(IndexerConstants.kRotatorStallDebounceSeconds, DebounceType.kRising);
  private final Alert disconnected;
  private final Alert rotatorDisconnected;
  private final Alert rotatorStalledAlert;

  private Debouncer atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput
  private boolean rotatorStalled = false;

  private RotatorRecoveryState rotatorRecoveryState = RotatorRecoveryState.NORMAL;
  private double normalRotatorVelocityRadsPerSec = 0.0;
  private double recoveryVelocityRadsPerSec = 0.0;
  private double commandedRotatorVelocityRadsPerSec = 0.0;
  private double outtakePhaseElapsedSecs = 0.0;
  private boolean outtakeReversePhase = true;
  private double recoveryElapsedSecs = 0.0;
  private int rotatorRecoveryAttempts = 0;

  public Indexer(IndexerIO io) {
    this.io = io;
    disconnected = new Alert("Indexer motor disconnected!", Alert.AlertType.kWarning);
    rotatorDisconnected = new Alert("Rotator motors disconnected!", Alert.AlertType.kWarning);
    rotatorStalledAlert =
        new Alert("Rotator stall detected; indexer stopped!", Alert.AlertType.kError);

    io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    io.setrotatorPID(
        kRotator_kP.get(),
        kRotator_kI.get(),
        kRotator_kD.get(),
        kRotator_kS.get(),
        kRotator_kV.get(),
        kRotator_kA.get(),
        kRotator_kG.get());
  }

  @Override
  public void updateInputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    updateTunables();
    disconnected.set(!motorConnectedDebouncer.calculate(inputs.connected));
    rotatorDisconnected.set(!rotatorConnectedDebouncer.calculate(inputs.rotatorconnected));
    updateRotatorStallDetection();
    rotatorStalledAlert.set(rotatorStalled);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      stopAllOutputs();
      cancelRotatorRecovery();
    } else {
      switch (goal) {
        case STOP -> {
          stopAllOutputs();
          cancelRotatorRecovery();
        }
        case INTAKE -> {
          stopAllOutputs();
          cancelRotatorRecovery();
        }
        case OUTTAKE -> {
          if (rotatorRecoveryState == RotatorRecoveryState.LOCKED_OUT) {
            stopAllOutputs();
          } else if (rotatorRecoveryState == RotatorRecoveryState.RECOVERING) {
            runRotatorRecovery();
          } else {
            runVelocityLogic(kOuttakeVelocity.get());
            runNormalRotatorVelocity(getOuttakeRotatorVelocity());
          }
        }
        case SHOOT, FORCE_SHOOT -> {
          if (rotatorRecoveryState == RotatorRecoveryState.LOCKED_OUT) {
            stopAllOutputs();
          } else if (rotatorRecoveryState == RotatorRecoveryState.RECOVERING) {
            runRotatorRecovery();
          } else {
            runVelocityLogic(kShootVelocity.get());
            runNormalRotatorVelocity(kRotatorShootVelocity.get());
          }
        }
      }
    }
  }

  @Override
  public void executePeriodic() {
    Logger.recordOutput("Indexer/Mode", outputs.mode);
    Logger.recordOutput("Indexer/Setpoint", outputs.velocityRadsPerSec);
    Logger.recordOutput("Indexer/RotatorMode", outputs.rotatorMode);
    Logger.recordOutput("Indexer/RotatorSetpoint", outputs.rotatorVelocityRadsPerSec);
    Logger.recordOutput("Indexer/RotatorRecoveryState", rotatorRecoveryState.name());
    Logger.recordOutput("Indexer/RotatorRecoveryAttempts", rotatorRecoveryAttempts);
    io.applyOutputs(outputs);
    Robot.batteryLogger.reportCurrentUsage(
        "Indexer", false, inputs.connected ? inputs.supplyCurrentAmps : 0.0);
  }

  /** Internal velocity closed-loop helper: sets output and computes atGoal */
  private void runVelocityLogic(double velocityRadsPerSec) {
    outputs.mode = IndexerIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = velocityRadsPerSec;

    boolean inTolerance =
        Math.abs(inputs.velocityRadsPerSec - velocityRadsPerSec) <= velocityTolerance.get();

    if (Math.abs(velocityRadsPerSec) < 1.0) {
      inTolerance = false;
    }

    atGoal = atGoalDebouncer.calculate(inTolerance);
  }

  private void runNormalRotatorVelocity(double velocityRadsPerSec) {
    normalRotatorVelocityRadsPerSec = velocityRadsPerSec;
    runRotatorVelocityLogic(velocityRadsPerSec);
  }

  private void runRotatorVelocityLogic(double velocityRadsPerSec) {
    double maxVelocityDelta =
        Math.max(0.0, kRotatorMaxAcceleration.get()) * Constants.loopPeriodSecs;
    commandedRotatorVelocityRadsPerSec +=
        MathUtil.clamp(
            velocityRadsPerSec - commandedRotatorVelocityRadsPerSec,
            -maxVelocityDelta,
            maxVelocityDelta);

    outputs.rotatorMode = IndexerIOOutputMode.VELOCITY;
    outputs.rotatorVelocityRadsPerSec = commandedRotatorVelocityRadsPerSec;

    boolean inTolerance =
        Math.abs(inputs.rotatorvelocityRadsPerSec - commandedRotatorVelocityRadsPerSec)
            <= velocityTolerance.get();
    if (Math.abs(commandedRotatorVelocityRadsPerSec) < 1.0) {
      inTolerance = false;
    }
    atGoalRotator = atGoalDebouncer.calculate(inTolerance);
  }

  private double getOuttakeRotatorVelocity() {
    double velocity =
        outtakeReversePhase ? -kRotatorOuttakeVelocity.get() : kRotatorOuttakeVelocity.get();
    double phaseDuration =
        outtakeReversePhase
            ? IndexerConstants.kRotatorOuttakeReverseSeconds
            : IndexerConstants.kRotatorOuttakeForwardSeconds;
    outtakePhaseElapsedSecs += Constants.loopPeriodSecs;
    if (outtakePhaseElapsedSecs >= phaseDuration) {
      outtakePhaseElapsedSecs -= phaseDuration;
      outtakeReversePhase = !outtakeReversePhase;
    }
    return velocity;
  }

  private void runRotatorRecovery() {
    stopIndexerOutput();
    runRotatorVelocityLogic(recoveryVelocityRadsPerSec);
    recoveryElapsedSecs += Constants.loopPeriodSecs;

    boolean movingInRecoveryDirection =
        Math.signum(inputs.rotatorvelocityRadsPerSec) == Math.signum(recoveryVelocityRadsPerSec)
            && Math.abs(inputs.rotatorvelocityRadsPerSec) > kRotatorStallVelocityThreshold.get();
    if (movingInRecoveryDirection) {
      finishRotatorRecovery(true);
    } else if (recoveryElapsedSecs >= IndexerConstants.kRotatorRecoverySeconds) {
      if (rotatorRecoveryAttempts >= IndexerConstants.kRotatorRecoveryMaxAttempts) {
        rotatorRecoveryState = RotatorRecoveryState.LOCKED_OUT;
        rotatorStalled = true;
        stopAllOutputs();
      } else {
        finishRotatorRecovery(false);
      }
    }
  }

  private void finishRotatorRecovery(boolean recovered) {
    rotatorRecoveryState = RotatorRecoveryState.NORMAL;
    recoveryElapsedSecs = 0.0;
    if (recovered) {
      rotatorRecoveryAttempts = 0;
    }
    rotatorStallDebouncer.calculate(false);
  }

  private void updateRotatorStallDetection() {
    if (rotatorRecoveryState != RotatorRecoveryState.NORMAL || !isRotatorRequested()) {
      rotatorStallDebouncer.calculate(false);
      return;
    }

    boolean atNormalRotatorTarget =
        Math.abs(outputs.rotatorVelocityRadsPerSec - normalRotatorVelocityRadsPerSec)
            <= kRotatorStallVelocityThreshold.get();
    boolean commandingRotator =
        outputs.rotatorMode == IndexerIOOutputMode.VELOCITY
            && Math.abs(outputs.rotatorVelocityRadsPerSec) > kRotatorStallVelocityThreshold.get()
            && atNormalRotatorTarget;
    boolean movingTooSlow =
        Math.abs(inputs.rotatorvelocityRadsPerSec) < kRotatorStallVelocityThreshold.get();
    boolean highTorque =
        Math.abs(inputs.rotatortorqueCurrentAmps) > kRotatorStallTorqueCurrentThreshold.get();

    if (rotatorStallDebouncer.calculate(commandingRotator && movingTooSlow && highTorque)) {
      beginRotatorRecovery();
    }

    Logger.recordOutput("Indexer/RotatorStallCommanding", commandingRotator);
    Logger.recordOutput("Indexer/RotatorStallAtNormalTarget", atNormalRotatorTarget);
    Logger.recordOutput("Indexer/RotatorStallMovingTooSlow", movingTooSlow);
    Logger.recordOutput("Indexer/RotatorStallHighTorque", highTorque);
  }

  private boolean isRotatorRequested() {
    return goal == IndexerGoal.OUTTAKE
        || goal == IndexerGoal.SHOOT
        || goal == IndexerGoal.FORCE_SHOOT;
  }

  private void beginRotatorRecovery() {
    if (Math.abs(normalRotatorVelocityRadsPerSec) < kRotatorStallVelocityThreshold.get()) {
      return;
    }

    rotatorRecoveryState = RotatorRecoveryState.RECOVERING;
    recoveryVelocityRadsPerSec = -normalRotatorVelocityRadsPerSec;
    recoveryElapsedSecs = 0.0;
    rotatorRecoveryAttempts++;
    rotatorStallDebouncer.calculate(false);
  }

  private void cancelRotatorRecovery() {
    rotatorRecoveryState = RotatorRecoveryState.NORMAL;
    normalRotatorVelocityRadsPerSec = 0.0;
    recoveryVelocityRadsPerSec = 0.0;
    commandedRotatorVelocityRadsPerSec = 0.0;
    outtakePhaseElapsedSecs = 0.0;
    outtakeReversePhase = true;
    recoveryElapsedSecs = 0.0;
    rotatorRecoveryAttempts = 0;
    rotatorStalled = false;
    rotatorStallDebouncer.calculate(false);
  }

  private void stopAllOutputs() {
    stopIndexerOutput();
    stopRotatorOutput();
    atGoal = false;
    atGoalRotator = false;
  }

  private void stopIndexerOutput() {
    outputs.mode = IndexerIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
  }

  private void stopRotatorOutput() {
    outputs.rotatorMode = IndexerIOOutputMode.COAST;
    outputs.rotatorVelocityRadsPerSec = 0.0;
    commandedRotatorVelocityRadsPerSec = 0.0;
  }

  /** Update tunable parameters */
  private void updateTunables() {
    if (atGoalDebounce.hasChanged(hashCode())) {
      atGoalDebouncer = new Debouncer(atGoalDebounce.get(), DebounceType.kFalling);
    }
    if (kP.hasChanged(hashCode())
        || kI.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kV.hasChanged(hashCode())
        || kA.hasChanged(hashCode())
        || kG.hasChanged(hashCode())) {
      io.setPID(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
    }
    if (kRotator_kP.hasChanged(hashCode())
        || kRotator_kI.hasChanged(hashCode())
        || kRotator_kD.hasChanged(hashCode())
        || kRotator_kS.hasChanged(hashCode())
        || kRotator_kV.hasChanged(hashCode())
        || kRotator_kA.hasChanged(hashCode())
        || kRotator_kG.hasChanged(hashCode())) {
      io.setrotatorPID(
          kRotator_kP.get(),
          kRotator_kI.get(),
          kRotator_kD.get(),
          kRotator_kS.get(),
          kRotator_kV.get(),
          kRotator_kA.get(),
          kRotator_kG.get());
    }
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  // --- Commands ---
  public Command setGoalCommand(IndexerGoal newGoal) {
    return Commands.runOnce(() -> this.goal = newGoal, this);
  }
}
