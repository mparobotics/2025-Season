package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoreAngle.ScoringPose;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class EWsetpoint extends SequentialCommandGroup {
    public EWsetpoint (ElevatorSubsystem elevator, WristSubsystem wrist, ScoringPose pose){
        addCommands(
            new ParallelCommandGroup(elevator.setSetpointCommand(pose.elevatorheight()),wrist.setSetpointCommand(pose.wristangle())),
            Commands.waitUntil(() -> wrist.isAtSetPoint() && elevator.isAtSetPoint())
            );
    }
    
}
