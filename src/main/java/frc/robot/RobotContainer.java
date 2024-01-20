// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File (Filesystem.getDeployDirectory(), "swerve"));

  // This code would go in a swerve subsystem, but we already defined one named m_swerve above
  //private double maximumSpeed = Units.feetToMeters(14.5);
  //private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  //private final SwerveDrive swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = m_swerve.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightY()
    );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRawAxis(2));

    m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }


  private void configureBindings() {
    m_driverController.x().whileTrue(
        m_swerve.run(()->{
            m_swerve.lock();})
    );
    m_driverController.a().whileTrue(
        m_swerve.run(()->{
            m_swerve.drive(new Translation2d(1, 0), 0.0, true);})
    );
    m_driverController.b().whileTrue(
        m_swerve.run(()->{
            m_swerve.drive(new Translation2d(0, 1), 0.0, true);})
    );
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
