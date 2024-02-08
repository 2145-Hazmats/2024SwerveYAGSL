// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestMotorSubsystem;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  private SendableChooser<Command> m_autonChooser;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_OperatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // It works bc it's a command that runs and runs and then it has a timeout so it can stop eventually.
    // PathPlanner doesn't stop the command or make it keep running, so you might want a run command. This one isn't a run command bc it doesn't need to be one.
    NamedCommands.registerCommand("SpinTheMotor", m_testMotor.TestStartEndCommand(0.8).withTimeout(1));

    m_swerve.setupPathPlanner();
    m_autonChooser = AutoBuilder.buildAutoChooser();

    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("Auton Picker", m_autonChooser);

    Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverController.getRawAxis(4));
    
    m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }


  private void configureBindings() {
    m_driverController.x().toggleOnTrue(
      m_swerve.run(()->{
        m_swerve.lock();
      })
    );

    m_driverController.a().whileTrue(
      m_testMotor.startEnd(
        ()->{
          m_testMotor.setMotor(0.3);
        }, 
        ()->{
          m_testMotor.setMotor(0);
        }
      )
    );

    m_driverController.y().onTrue(
      m_swerve.runOnce(()->{
        m_swerve.resetGyro();
      })
    );

    // Use path I created
    m_driverController.b().onTrue(
      m_swerve.runOnce(() -> {
        m_swerve.pathFindingCommand();
      })
    );
  }

  
  public void SetMotorBrake(boolean brake) {
    m_swerve.setMotorBrake(brake);
  }


  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }
}
