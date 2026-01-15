// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Command.AutoAlign;
import frc.robot.Command.TeleopSwerve;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private final CommandXboxController driveController = new CommandXboxController(0);  
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric = new Trigger(driveController.leftBumper());

  private final SwerveSubsystem m_drive = new SwerveSubsystem();

  public RobotContainer() {
    configureBindings();}
  private void configureBindings() {
    //driveController.button(Button.kLeftBumper.value).whileTrue (m_ClimberSubsystem.InverseMotors().repeatedly());
    //driveController.button(Button.kRightBumper.value).whileTrue (m_ClimberSubsystem.RunMotors().repeatedly());
    driveController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_drive.zeroGyro(), m_drive));
    driveController.axisGreaterThan(Axis.kLeftTrigger.value, 0.1).whileTrue(new AutoAlign(m_drive, true));
    driveController.axisGreaterThan(Axis.kRightTrigger.value, 0.1).whileTrue(new AutoAlign(m_drive, false));


    m_drive.setDefaultCommand(
    new TeleopSwerve(
        m_drive,
        () -> -getSpeedMultiplier() * driveController.getRawAxis(translationAxis) * 0.5, // * to change drive speed
        () -> -getSpeedMultiplier() * driveController.getRawAxis(strafeAxis) * 0.5, // * to change drive speed
        () -> -driveController.getRawAxis(rotationAxis) * 0.5, // * to change turn speed //put - infront of drivecontroller of take it away to tune the turning
        () -> robotCentric.getAsBoolean(),
        () -> driveController.getRightTriggerAxis() > 0.1
        //() -> driveController.getHID().getRawButton(button.kX.value)
        
  ));

  }


  private double getSpeedMultiplier(){
    return driveController.getHID().getRawButton(Button.kLeftStick.value)? 0.7: 1; //can change speed
  }

  
}