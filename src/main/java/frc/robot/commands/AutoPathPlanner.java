// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;


public class AutoPathPlanner extends Command {
  private final SwerveSubsystem m_swerve;
  private String pathName;


  // Constructor
  public AutoPathPlanner(SwerveSubsystem subsystem, String pathName) {
    m_swerve = subsystem;
    
    addRequirements(m_swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: Get max speeds and basic swerve properties and load into the pathplanner program
    m_swerve.getAutonomousCommand(pathName, true);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
