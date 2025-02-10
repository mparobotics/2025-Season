// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final CommandXboxController driveController = new CommandXboxController(0); 
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(); 
  private final CommandXboxController helmsController = new CommandXboxController(1);
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric = new Trigger(driveController.leftBumper());

  private final SwerveSubsystem m_drive = new SwerveSubsystem();

  public RobotContainer() {
    configureBindings();}
  private void configureBindings() {
    m_ElevatorSubsystem.setDefaultCommand(m_ElevatorSubsystem.Run1(() -> helmsController.getLeftY()));

    m_IntakeSubsystem.setDefaultCommand(m_IntakeSubsystem.Run1(() -> helmsController.getRightY()));

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