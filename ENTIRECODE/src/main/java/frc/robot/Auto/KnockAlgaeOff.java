package frc.robot.Auto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Command.EWsetpoint;
import frc.robot.Constants.ScoreAngle;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class KnockAlgaeOff extends SequentialCommandGroup{
    public KnockAlgaeOff (SwerveSubsystem drive, ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake) {
        addCommands(
            drive.startAutoAt(7.13, 7.276, 180),
            drive.autoDrive("One Coral Path"),
            new EWsetpoint(elevator, wrist, ScoreAngle.L1),
            intake.RunIntake(() -> 1),
            new EWsetpoint(elevator, wrist, ScoreAngle.KnockAlgae),
            intake.RunIntake(() -> -1),
            drive.autoDrive("AlgaeOne"),
            new EWsetpoint(elevator, wrist, ScoreAngle.KnockAlgae),
            intake.RunIntake(() -> 1),
            drive.autoDrive("AlgaeTwo"),
            new EWsetpoint(elevator, wrist, ScoreAngle.KnockAlgae),
            intake.RunIntake(() -> -1),
            drive.autoDrive("AlgaeThree"),
            new EWsetpoint(elevator, wrist, ScoreAngle.KnockAlgae),
            intake.RunIntake(() -> 1),
            drive.autoDrive("AlgaeFour"),
            new EWsetpoint(elevator, wrist, ScoreAngle.KnockAlgae),
            intake.RunIntake(() -> -1),
            drive.autoDrive("AlgaeFive"),
            new EWsetpoint(elevator, wrist, ScoreAngle.KnockAlgae),
            intake.RunIntake(() -> 1),
            drive.autoDrive("AlgaeFive to Source"),
            new EWsetpoint(elevator, wrist, ScoreAngle.INTAKE),
            intake.RunIntake(() -> -1),
            intake.RunIntake(() -> 1)

        );
    }
}
