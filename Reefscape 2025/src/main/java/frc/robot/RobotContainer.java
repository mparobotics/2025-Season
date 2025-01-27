// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.MotorSubsystem;

public class RobotContainer {
  
  private final MotorSubsystem m_MotorSubsystem = new MotorSubsystem();
  private final CommandXboxController controller = new CommandXboxController(0);
  //basic classes and private finals

   
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    m_MotorSubsystem.setDefaultCommand(m_MotorSubsystem.Run1(() -> controller.getRightY()));
    
    controller.axisGreaterThan(Axis.kRightY.value,0.2).whileTrue(m_MotorSubsystem.RunMotors(() -> Axis.kRightY.value).repeatedly());
    controller.axisLessThan(Axis.kRightY.value, -0.2).whileTrue(m_MotorSubsystem.RunMotors(() -> Axis.kRightY.value).repeatedly());
    m_MotorSubsystem.setDefaultCommand(m_MotorSubsystem.StopMotors());
  }
  /* 
    controller.axisGreaterThan(Axis.kRightY.value, 0.5).whileTrue(m_MotorSubsystem.RunMotors().repeatedly());
    controller.axisLessThan(Axis.kRightY.value, -0.5).whileTrue(m_MotorSubsystem.InverseMotors().repeatedly());*/
   // m_MotorSubsystem.setDefaultCommand(((m_MotorSubsystem.StopMotors())));
  
  //to run the move motor commands in MotorSubsystems

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured"); 

  }

  public void periodic() {

    SmartDashboard.putNumber("Controller Axis Value", controller.getRightY());
  }
}
