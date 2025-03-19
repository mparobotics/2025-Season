// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.WristSubsystem;

public class RobotContainer {
  private final CommandXboxController driveController = new CommandXboxController(0); 
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(); 
  private final CommandXboxController helmsController = new CommandXboxController(1);
   private final WristSubsystem m_WristSubsystem = new WristSubsystem();
    private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem(); 
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric = new Trigger(driveController.leftBumper());

  private final SwerveSubsystem m_drive = new SwerveSubsystem();

  public RobotContainer() {
    configureBindings();}
  private void configureBindings() {
    driveController.button(Button.kLeftBumper.value).whileTrue (m_ClimberSubsystem.InverseMotors().repeatedly());
    driveController.button(Button.kRightBumper.value).whileTrue (m_ClimberSubsystem.RunMotors().repeatedly());
    driveController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_drive.zeroGyro(), m_drive));

    //driveController.button(Button.kX.value).onTrue()

    SmartDashboard.putData("set setpoint to 0", m_ElevatorSubsystem.setSetpointCommand(0));
    SmartDashboard.putData("set setpoint to 0.5", m_ElevatorSubsystem.setSetpointCommand(0.5));

    SmartDashboard.putData("set wrist setpoint to 0", m_WristSubsystem.setSetpointCommand(0));
    SmartDashboard.putData("set wrist setpoint to 90", m_WristSubsystem.setSetpointCommand(90));


    //helmsController.axisGreaterThan(Axis.kRightY.value, 0.5).whileTrue(m_ElevatorSubsystem.RunMotors().repeatedly());
    //helmsController.axisLessThan(Axis.kRightY.value, -0.5).whileTrue(m_ElevatorSubsystem.InverseMotors().repeatedly());
    helmsController.povDown().whileTrue(m_WristSubsystem.InverseWrist().repeatedly()); 
    helmsController.povUp().whileTrue(m_WristSubsystem.RunWrist().repeatedly());
    helmsController.a().onTrue(m_ElevatorSubsystem.setSetpointCommand(0.1)); //in meters
    helmsController.b().onTrue(m_ElevatorSubsystem.setSetpointCommand(0));
    helmsController.x().onTrue(m_ElevatorSubsystem.setSetpointCommand(0.25));
    helmsController.y().onTrue(m_ElevatorSubsystem.setSetpointCommand(0.5));
    
    //helmsController.
    //m_ElevatorSubsystem.setDefaultCommand(((m_ElevatorSubsystem.StopMotors())));
    m_IntakeSubsystem.setDefaultCommand(m_IntakeSubsystem.RunIntake(() -> helmsController.getLeftY()));
    m_ClimberSubsystem.setDefaultCommand(((m_ClimberSubsystem.StopMotors())));
    //m_WristSubsystem.setDefaultCommand(m_WristSubsystem.RunIntake(() -> helmsController.getRightY()));


    m_drive.setDefaultCommand(
    new TeleopSwerve(
        m_drive,
        () -> -getSpeedMultiplier() * driveController.getRawAxis(translationAxis),
        () -> -getSpeedMultiplier() * driveController.getRawAxis(strafeAxis),
        () -> -driveController.getRawAxis(rotationAxis),
        () -> robotCentric.getAsBoolean(),
        () -> driveController.getRightTriggerAxis() > 0.1
        //() -> driveController.getHID().getRawButton(button.kX.value)
        
  ));

  }


  private double getSpeedMultiplier(){
    return driveController.getHID().getRawButton(Button.kLeftStick.value)? 0.7: 1;
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(m_drive.setStartingPose(new Pose2d(7.130, 7.276, Rotation2d.fromDegrees(-175.210))),
    m_drive.autoDrive("Example Path"));
  }

}