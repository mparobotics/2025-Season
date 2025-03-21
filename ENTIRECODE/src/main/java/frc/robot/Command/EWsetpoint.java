package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ScoreAngle.ScoringPose;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class EWsetpoint extends SequentialCommandGroup {
    public EWsetpoint (ElevatorSubsystem elevator, WristSubsystem wrist, ScoringPose pose){
        if (pose.wristangle() < 0){
            addCommands(elevator.setSetpointCommand(pose.elevatorheight()),
            Commands.waitUntil(() -> elevator.isElevatorHighEnough()),
            wrist.setSetpointCommand(pose.wristangle()));
        }
        else{
            addCommands(elevator.setSetpointCommand(pose.elevatorheight()),
            wrist.setSetpointCommand(pose.wristangle()));
        }


        addCommands(
            Commands.waitUntil(() -> wrist.isAtSetPoint() && elevator.isAtSetPoint())
            );
    }
    
}
