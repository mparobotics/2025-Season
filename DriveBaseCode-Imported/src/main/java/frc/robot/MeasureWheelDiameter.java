// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystens.SwerveSubsystem;

public class MeasureWheelDiameter extends Command {
  /** Creates a new MeasureWheelDiameter. */
   private SwerveSubsystem m_drive;
  private Rotation2d startAngle;
  private double[] startPositions;
  private double[] distances = {0,0,0,0};
  /** Creates a new MeasureWheelDiameter. */
  public MeasureWheelDiameter(SwerveSubsystem drive) {
    addRequirements(drive);
    m_drive = drive;}
  public MeasureWheelDiameter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { startPositions = new double[4];
    startAngle = m_drive.getYaw();

    double[] modules = m_drive.getEncoderRotations();
    for(var i = 0; i < modules.length; i++){
      startPositions[i] = modules[i];
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(new Translation2d(0,0),0.25,false,false);
    double[] modules = m_drive.getEncoderRotations();
    for(var i = 0; i < modules.length; i++){
      distances[i] = modules[i] - startPositions[i];
    }
    double distanceTraveled = SwerveConstants.driveBaseRadius * m_drive.getYaw().minus(startAngle).getRadians();
    
    double avgRotations = 0;
    for(var i = 0; i < distances.length; i++){
      avgRotations += distances[i];
    }
    avgRotations /= distances.length;
    double calculatedWheelCircumference = avgRotations / distanceTraveled;
    double calculatedWheelDiameter = calculatedWheelCircumference / 2 * Math.PI;
    SmartDashboard.putNumber("Wheel Diameter", calculatedWheelDiameter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(new Translation2d(0,0),0.0,false,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
