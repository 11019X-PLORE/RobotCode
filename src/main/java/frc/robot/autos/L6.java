package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.extension.Extension;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.superstructure.SuperstructureFactory;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class L6 extends SequentialCommandGroup {
  /** Extra clearance along the field X axis, applied only while L6 is passing in autonomous. */
  private static final double kAutoPassTrenchMarginMeters = 1.3;

  public L6(RobotContainer c, Supplier<Boolean> isRightSide) {
    Intake intake = c.getIntake();
    Turret turret = c.getTurret();
    Hood hood = c.getHood();
    Extension extension = c.getExtension();
    Drive drive = c.getDrive();

    var autoRoutine =
        Commands.sequence(
            Commands.runOnce(() -> drive.setYFlipped(isRightSide)),
            Commands.parallel(turret.zeroCommand(), hood.zeroCommand(), extension.zeroCommand()),
            drive.resetOdomToPath("L1"),
            Commands.deadline(
                drive.generatePath("L1"),
                intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
                extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
                Commands.parallel(
                    SuperstructureFactory.pass(c),
                    hood.autoTrench(drive::getPose, kAutoPassTrenchMarginMeters)),
                SuperstructureFactory.forceFeeding(c)),
            Commands.deadline(
                drive.generatePath("L2"),
                hood.autoTrench(drive::getPose, kAutoPassTrenchMarginMeters)),
            intake.setGoalCommand(Intake.IntakeGoal.STOP).withTimeout(0.1),
            Commands.deadline(
                SuperstructureFactory.shoot(c).withTimeout(5.0),
                SuperstructureFactory.forceFeeding(c),
                intake.setGoalCommand(Intake.IntakeGoal.STOW),
                extension.setGoalCommand(Extension.ExtensionGoal.SHAKE)),
            Commands.parallel(
                    SuperstructureFactory.autoIntake(c), SuperstructureFactory.stopFeeding(c))
                .withTimeout(0.1),
            Commands.deadline(
                drive.generatePath("L14"),
                intake.setGoalCommand(Intake.IntakeGoal.INTAKE),
                extension.setGoalCommand(Extension.ExtensionGoal.DEPLOYED),
                SuperstructureFactory.autoIntake(c),
                SuperstructureFactory.runIndexerIntake(c)),
            intake.setGoalCommand(Intake.IntakeGoal.STOP).withTimeout(0.1),
            Commands.parallel(
                SuperstructureFactory.shoot(c),
                SuperstructureFactory.feeding(c),
                intake.setGoalCommand(Intake.IntakeGoal.STOW),
                extension.setGoalCommand(Extension.ExtensionGoal.SHAKE)));

    addCommands(
        Commands.deadline(
            autoRoutine,
            Commands.run(
                () ->
                    Logger.recordOutput(
                        "Auto/L6/InExpandedTrench",
                        hood.isInAutoTrench(drive::getPose, kAutoPassTrenchMarginMeters)))));
  }
}
