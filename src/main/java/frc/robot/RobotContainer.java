// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SpinMotorCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestMotorSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File (Filesystem.getDeployDirectory(), "swerve"));
  private final TestMotorSubsystem m_testMotor = new TestMotorSubsystem();
  

  private SendableChooser<Command> m_autonChooser = AutoBuilder.buildAutoChooser();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_OperatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    NamedCommands.registerCommand("Spin Motor", 
      Commands.run(
        ()->{
          m_testMotor.setMotor(0.4);
        }, 
      m_testMotor));

    NamedCommands.registerCommand("print message", Commands.print("hellaur :3"));

    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("Auton Picker", m_autonChooser);

    Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverController.getRawAxis(4));
    
    // Idea that counts but not very good
    Command drive2 = m_swerve.driveCommand(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND));

    //m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    /*
    Command driveFieldOrientedDirectAngleSim = m_swerve.simDriveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverController.getRawAxis(4));
    */

    m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    //m_swerve.setDefaultCommand(
    //    !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

    //NamedCommands.registerCommand("PlaySpeakerCommand", new PlaySpeakerCommand(m_testMotor, 0.8).withTimeout(5));
  }


  private void configureBindings() {
    m_driverController.x().toggleOnTrue(
        m_swerve.run(()->{
            m_swerve.lock();})
    );

    m_driverController.a().whileTrue(
      m_testMotor.startEnd(
        ()->{
          m_testMotor.setMotor(0.3);
        }, 
        ()->{
          m_testMotor.setMotor(0);
        }));

    m_driverController.y().onTrue(
      m_swerve.runOnce(()->{
        m_swerve.resetGyro();
      }));
  }

  
  public void SetMotorBrake(boolean brake) {
    m_swerve.setMotorBrake(brake);
  }


  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }
}
