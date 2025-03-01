// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    configureBindings();
    m_drive.resetOdometry(new Pose2d());
  }
  private void configureBindings() {
    //m_ElevatorSubsystem.setDefaultCommand(m_ElevatorSubsystem.Run1(() -> helmsController.getLeftY()));
    helmsController.axisGreaterThan(Axis.kRightY.value, 0.5).whileTrue(m_ElevatorSubsystem.RunMotors().repeatedly());
    helmsController.axisLessThan(Axis.kRightY.value, -0.5).whileTrue(m_ElevatorSubsystem.InverseMotors().repeatedly());
    helmsController.button(Button.kLeftBumper.value).whileTrue(m_ClimberSubsystem.InverseMotors().repeatedly());
    helmsController.button(Button.kRightBumper.value).whileTrue(m_ClimberSubsystem.RunMotors().repeatedly());
    helmsController.povDown().whileTrue(m_WristSubsystem.InverseMotors().repeatedly()); 
    helmsController.povUp().whileTrue(m_WristSubsystem.RunMotors().repeatedly());
    m_ElevatorSubsystem.setDefaultCommand(((m_ElevatorSubsystem.StopMotors())));
    m_IntakeSubsystem.setDefaultCommand(m_IntakeSubsystem.Run1(() -> helmsController.getLeftY()));
    m_WristSubsystem.setDefaultCommand(((m_WristSubsystem.StopMotors())));
    m_ClimberSubsystem.setDefaultCommand(((m_ClimberSubsystem.StopMotors())));


    m_drive.setDefaultCommand(
    new TeleopSwerve(
        m_drive,
        () -> -getSpeedMultiplier() * driveController.getRawAxis(translationAxis),
        () -> -getSpeedMultiplier() * driveController.getRawAxis(strafeAxis),
        () -> -driveController.getRawAxis(rotationAxis),
        () -> robotCentric.getAsBoolean()
        //() -> driveController.getHID().getRawButton(button.kX.value)
        
  ));

  }


  private double getSpeedMultiplier(){
    return driveController.getHID().getRawButton(Button.kLeftStick.value)? 0.7: 1;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}