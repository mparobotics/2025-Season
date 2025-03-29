package frc.robot.Auto;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Command.EWsetpoint;
import frc.robot.Constants.ScoreAngle;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class OneCoralAutoL2 extends SequentialCommandGroup{
    public OneCoralAutoL2 (SwerveSubsystem drive, ElevatorSubsystem elevator, WristSubsystem wrist, IntakeSubsystem intake) {
        addCommands(
            drive.startAutoAt(8.079, 7.288, 0),
            drive.autoDrive("One Coral Path L2"),
            Commands.waitSeconds(1),
            new EWsetpoint(elevator, wrist, ScoreAngle.L2),
            intake.RunIntake(() -> 1)
            //new EWsetpoint(elevator, wrist, ScoreAngle.KnockAlgae),
            //intake.RunIntake(() -> -1)
        );
    }
}
