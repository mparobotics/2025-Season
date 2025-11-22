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
    private SwerveSubsystem m_SwerveSubsystem; // Drivetrain used to command chassis motion

    private PIDController xController = new PIDController(0.8, 0, 0);
    private PIDController yController = new PIDController(0.8, 0, 0);
    private PIDController rotationController = new PIDController(0.005, 0, 0);
    private boolean isLeft;

    private ChassisSpeeds getAutoAlignSpeed (Pose2d CurrentPosition, Translation2d ReefCenter, double DistanceFromReef, double BranchDistancefromMiddle){
        if (isLeft){
            BranchDistancefromMiddle *= -1; // Mirror branch offset when approaching left side
        }
        Translation2d OffSet = CurrentPosition.getTranslation().minus(ReefCenter); // Current vector from reef center
        double goalAngle = Math.round((OffSet.getAngle().getDegrees())/60) * 60; // Snap orientation to nearest 60° branch
        Rotation2d goalRotation = Rotation2d.fromDegrees(goalAngle); // Convert snapped angle to Rotation2d
        Translation2d scoringLocation = new Translation2d(DistanceFromReef, BranchDistancefromMiddle); // Target offset from reef center
        scoringLocation = scoringLocation.rotateBy(goalRotation); // Rotate target into correct branch quadrant
        scoringLocation = scoringLocation.plus(ReefCenter); // Translate back into field coordinates

        double xOutput = xController.calculate(CurrentPosition.getX(), scoringLocation.getX()); // Field X correction
        double yOutput = yController.calculate(CurrentPosition.getY(), scoringLocation.getY()); // Field Y correction
        double rotationOutput = rotationController.calculate(CurrentPosition.getRotation().getDegrees(), goalAngle + 180); // Heading correction (face reef)
        return new ChassisSpeeds(xOutput, yOutput, rotationOutput); // Desired chassis motion in field frame
    } 

    public void execute(){
        ChassisSpeeds ssppeeeedd = getAutoAlignSpeed(m_SwerveSubsystem.getPose(), FieldConstants.flipForAlliance(FieldConstants.BLUE_REEF_CENTER), 
        1.55, 0.2); // Compute desired auto-alignment velocity
        m_SwerveSubsystem.driveFromChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(ssppeeeedd, m_SwerveSubsystem.getPose().getRotation()), true);
    }

    public AutoAlign(SwerveSubsystem drive, boolean Left){
        m_SwerveSubsystem = drive; // Store drivetrain reference
        isLeft = Left; // Remember which branch side to target
        rotationController.enableContinuousInput (-180, 180); // Allow heading PID to wrap across ±180°
        addRequirements(m_SwerveSubsystem); // Reserve drivetrain while auto-align runs
    }

}

