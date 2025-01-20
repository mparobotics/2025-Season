// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

<<<<<<< Updated upstream
  private void configureBindings() {}
=======
  private void configureBindings() {

    /*controller.axisGreaterThan(Axis.kRightY.value, 0.5).whileTrue(m_MotorSubsystem.RunMotors().repeatedly());
    controller.axisLessThan(Axis.kRightY.value, 0.5).whileTrue(m_MotorSubsystem.InverseMotors().repeatedly());
    m_MotorSubsystem.setDefaultCommand(((m_MotorSubsystem.StopMotors())));*/
    m_MotorSubsystem.setDefaultCommand(m_MotorSubsystem.SetMotor(controller.getLeftY()));
  }
  //to run the move motor commands in MotorSubsystems
>>>>>>> Stashed changes

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
