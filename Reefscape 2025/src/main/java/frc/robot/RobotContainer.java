// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.ClimbSubsystem;

public class RobotContainer {

  private final ClimbSubsystem m_Climb = new ClimbSubsystem();

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.axisGreaterThan(Axis.kLeftY.value, 0.1).whileTrue(m_Climb.changeArmSetpointCommand(-1).repeatedly());
    controller.axisLessThan(Axis.kLeftY.value, -0.1).whileTrue(m_Climb.changeArmSetpointCommand(1).repeatedly());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
