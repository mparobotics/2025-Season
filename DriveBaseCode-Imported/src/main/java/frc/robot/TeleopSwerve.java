// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;

public class TeleopSwerve extends Command {
  private SwerveSubsystem m_SwerveSubsystem;
  private DoubleSupplier m_xSupplier, m_ySupplier, m_rotationSupplier;




  private SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);  // lower to two if robot unstable  
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(SwerveSubsystem SwerveSubsystem,
  DoubleSupplier xSupplier,
  DoubleSupplier ySupplier,
  DoubleSupplier rotationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = SwerveSubsystem;
    addRequirements(m_SwerveSubsystem);
    this.m_xSupplier = xSupplier;
    this.m_ySupplier = ySupplier;
    this.m_rotationSupplier = rotationSupplier;
  }
  @Override
  public void execute() {
    //if we're on red alliance, translation values are inverted since the drivers face the opposite direction
    int invert = FieldConstants.isRedAlliance()? -1: 1;
    
    double xVal = invert * 
        xLimiter.calculate(
            MathUtil.applyDeadband(m_xSupplier.getAsDouble(), SwerveConstants.inputDeadband));
    double yVal = invert * 
        yLimiter.calculate(
            MathUtil.applyDeadband(m_ySupplier.getAsDouble(), SwerveConstants.inputDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), SwerveConstants.inputDeadband));
    
    boolean isFieldOriented = !m_robotCentricSupplier.getAsBoolean();


    double currentDirection = m_SwerveSubsystem.getPose().getRotation().getDegrees();

   
      //If we're not auto aiming, just drive normally
      /* Drive */
      m_SwerveSubsystem.drive(
      //the joystick values (-1 to 1) multiplied by the max speed of the drivetrain
      xVal * SwerveConstants.maxSpeed, 
      yVal * SwerveConstants.maxSpeed, 
      rotationVal * SwerveConstants.maxAngularVelocity, 
      isFieldOriented);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
  