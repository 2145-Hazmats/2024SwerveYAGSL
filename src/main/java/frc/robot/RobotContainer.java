// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoPathPlanner;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File (Filesystem.getDeployDirectory(), "swerve"));

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_OperatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRawAxis(4));

    m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }


  private void configureBindings() {
    m_driverController.x().toggleOnTrue(
        m_swerve.run(()->{
            m_swerve.lock();})
    );

  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new AutoPathPlanner(m_swerve, "auto1"); // Pass different file names for different paths
  }
}
