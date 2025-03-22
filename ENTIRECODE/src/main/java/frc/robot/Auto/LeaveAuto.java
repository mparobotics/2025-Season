package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.SwerveSubsystem;

public class LeaveAuto extends SequentialCommandGroup{
    public LeaveAuto (SwerveSubsystem drive) {
        addCommands(
            new InstantCommand(() -> drive.drive(-1,0,0, false), drive),
            Commands.waitSeconds(2),
            new InstantCommand(() -> drive.drive(0,0,0, false), drive)

        );
    }

    
}