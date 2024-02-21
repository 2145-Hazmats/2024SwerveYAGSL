// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BoxSubsystem;

public class IntakeCommand extends Command {

  private final ArmSubsystem m_arm;
  private final BoxSubsystem m_box;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(ArmSubsystem armSubsystem, BoxSubsystem boxSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
    addRequirements(boxSubsystem);
    m_arm = armSubsystem;
    m_box = boxSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmPIDCommand(ArmPosition.FLOOR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_arm.getWristEncoder() >= 20 && m_box.getIntakeVelocity() <= .95) {
      m_box.setIntakeMotorCommand(Constants.BoxConstants.kIntakeSpeed);
    }

    if (m_arm.getWristEncoder() >= 37.8) {
      m_arm.PIDFallin();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_box.isReverseLimitSwitchPressed());
  }
}
