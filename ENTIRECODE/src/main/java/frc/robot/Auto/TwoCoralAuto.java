package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Command.EWsetpoint;
import frc.robot.Constants.ScoreAngle;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class TwoCoralAuto extends SequentialCommandGroup{
    public TwoCoralAuto (SwerveSubsystem drive, ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake) {
        addCommands(
            drive.startAutoAt(7.13, 7.276, 180),
            drive.autoDrive("One Coral Path"),
            new EWsetpoint(elevator, wrist, ScoreAngle.L1),
            intake.RunIntake(() -> 1),
            new EWsetpoint(elevator, wrist, ScoreAngle.KnockAlgae),
            intake.RunIntake(() -> -1),

            drive.autoDrive("Two Coral to Source"),
            Commands.waitSeconds(3),
            intake.RunIntake(() -> 1), //do wait and run intake at the same time
            drive.autoDrive("Source to Two Coral"),
            new EWsetpoint(elevator, wrist, ScoreAngle.L3),
            intake.RunIntake(() -> -1)
        );
    }
}
