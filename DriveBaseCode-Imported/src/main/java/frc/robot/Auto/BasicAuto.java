// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;

public class BasicAuto extends Command {
  /** Creates a new BasicAuto. */
 
  private SwerveSubsystem m_drive;

  public BasicAuto(SwerveSubsystem drive) {
    m_drive = drive;

    m_drive.driveForewardCommand();
  }
}