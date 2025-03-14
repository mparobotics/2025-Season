// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class TeleopSwerve extends Command {
  private SwerveSubsystem m_SwerveSubsystem;
  private DoubleSupplier m_translationSupplier;
  private DoubleSupplier m_strafeSupplier;
  private DoubleSupplier m_rotationSupplier;
  private BooleanSupplier m_robotCentricSupplier;
  private BooleanSupplier isAutoAlignSupplier;

  private PIDController autoAlignController = new PIDController(0, 0, 0); //change later

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0); //can only change by 3 m/s in the span of 1 s
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(SwerveSubsystem SwerveSubsystem,
      DoubleSupplier translationSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier,
      BooleanSupplier isAutoAlignSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = SwerveSubsystem;
    addRequirements(m_SwerveSubsystem);
    this.m_translationSupplier = translationSupplier;
    this.m_strafeSupplier = strafeSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_robotCentricSupplier = robotCentricSupplier;
    this.isAutoAlignSupplier = isAutoAlignSupplier;

    autoAlignController.enableContinuousInput(-180, 180);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        /* Get Values, applies Deadband, (doesnt do anything if stick is less than a value)*/
    double xVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(m_translationSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));
    double yVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(m_strafeSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));
    int invert = 1;
      if (FieldConstants.isRedAlliance()){
        invert = -1;
      }
  if (isAutoAlignSupplier.getAsBoolean()){
    double AngletoReef = 
    m_SwerveSubsystem.getPose().getTranslation().minus(FieldConstants.flipForAlliance(FieldConstants.BLUE_REEF_CENTER)).getAngle().getDegrees();
    
    double goalAngle;
    if (AngletoReef < -150){
      goalAngle = 0;
    }
    else if (AngletoReef < -90){
      goalAngle = 60;
    }
    else if (AngletoReef < -30){
      goalAngle = 120;
    }
    else if (AngletoReef < 30){
      goalAngle = 180;
    }
    else if (AngletoReef < 90){
      goalAngle = 240;
    }
    else if (AngletoReef < 150){
      goalAngle = 300;
    }
    else{
      goalAngle = 0;
    }

    SmartDashboard.putNumber("autoalign goalAngle", goalAngle);

    rotationVal = autoAlignController.calculate(m_SwerveSubsystem.getPose().getRotation().getDegrees(), goalAngle);
  }


    /* Drive */
    m_SwerveSubsystem.drive(
        //the joystick values (-1 to 1) multiplied by the max speed of the drivetrain
        xVal * SwerveConstants.maxSpeed * invert, yVal * SwerveConstants.maxSpeed * invert,
        //rotation value times max spin speed
        rotationVal * Constants.SwerveConstants.maxAngularVelocity,
        //whether or not in field centric mode
        !m_robotCentricSupplier.getAsBoolean());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}