package frc.robot.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
    private SwerveSubsystem m_SwerveSubsystem;

    private PIDController xController = new PIDController(0.8, 0, 0);
    private PIDController yController = new PIDController(0.8, 0, 0);
    private PIDController rotationController = new PIDController(0.005, 0, 0);
    private boolean isLeft;

    private ChassisSpeeds getAutoAlignSpeed (Pose2d CurrentPosition, Translation2d ReefCenter, double DistanceFromReef, double BranchDistancefromMiddle){
        if (isLeft){
            BranchDistancefromMiddle *= -1;
        }
        Translation2d OffSet = CurrentPosition.getTranslation().minus(ReefCenter);
        double goalAngle = Math.round((OffSet.getAngle().getDegrees())/60) * 60;
        Rotation2d goalRotation = Rotation2d.fromDegrees(goalAngle);
        Translation2d scoringLocation = new Translation2d(DistanceFromReef, BranchDistancefromMiddle);
        scoringLocation = scoringLocation.rotateBy(goalRotation);
        scoringLocation = scoringLocation.plus(ReefCenter);

        double xOutput = xController.calculate(CurrentPosition.getX(), scoringLocation.getX());
        double yOutput = yController.calculate(CurrentPosition.getY(), scoringLocation.getY());
        double rotationOutput = rotationController.calculate(CurrentPosition.getRotation().getDegrees(), goalAngle + 180);
        return new ChassisSpeeds(xOutput, yOutput, rotationOutput);
    } 

    public void execute(){
        ChassisSpeeds ssppeeeedd = getAutoAlignSpeed(m_SwerveSubsystem.getPose(), FieldConstants.flipForAlliance(FieldConstants.BLUE_REEF_CENTER), 
        1.55, 0.2);
        m_SwerveSubsystem.driveFromChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(ssppeeeedd, m_SwerveSubsystem.getPose().getRotation()), true);
    }

    public AutoAlign(SwerveSubsystem drive, boolean Left){
        m_SwerveSubsystem = drive;
        isLeft = Left;
        rotationController.enableContinuousInput (-180, 180);
        addRequirements(m_SwerveSubsystem);
    }

}


