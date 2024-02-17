// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.*;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File (Filesystem.getDeployDirectory(), "swerve"));
  private final BoxSubsystem m_box = new BoxSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  private SendableChooser<Command> m_autonChooser;

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_swerve.setupPathPlanner();
    m_autonChooser = AutoBuilder.buildAutoChooser();

    NamedCommands.registerCommand("MoveArm", m_arm.setArmPIDCommand(10, 0).withTimeout(1));

    SmartDashboard.putData("Auton Picker", m_autonChooser);
  
    m_swerve.setDefaultCommand(m_swerve.driveCommandAngularVelocity(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverController.getRawAxis(4),
        Constants.OperatorConstants.kFastModeSpeed
    ));
  }


  private void configureBindings() {
    /* Driver Controls */
    // Locks the wheels
    m_driverController.start().toggleOnTrue(
      m_swerve.run(()->{
        m_swerve.lock();
      })
    );

    m_driverController.a().onTrue(m_swerve.driveCommandPoint(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> 0,
      () -> -1
    ).until(() -> Math.abs(m_driverController.getRightX()) >= OperatorConstants.RIGHT_X_DEADBAND));
    
    m_driverController.b().whileTrue(m_swerve.driveCommandPoint(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> 1,
      () -> 0
    ));

    m_driverController.x().whileTrue(m_swerve.driveCommandPoint(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -1,
      () -> 0
    ));

    m_driverController.y().whileTrue(m_swerve.driveCommandPoint(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> 0,
      () -> 1
    ));

    // Resets the gyro
    m_driverController.back().onTrue(
      m_swerve.runOnce(()->{
        m_swerve.resetGyro();
      })
    );
 
    m_driverController.rightTrigger().whileTrue(
      m_swerve.driveCommandAngularVelocity(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverController.getRawAxis(4),
        OperatorConstants.kMidModeSpeed
      )
    );
  
    m_driverController.leftTrigger().whileTrue(
      m_swerve.driveCommandAngularVelocity(
        () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -m_driverController.getRawAxis(4),
        OperatorConstants.kSlowModeSpeed
      )
    );

    m_driverController.rightBumper().toggleOnTrue(m_swerve.driveCommandPoint(
      () -> MathUtil.applyDeadband(-m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-m_driverController.getRightY(), OperatorConstants.LEFT_X_DEADBAND)
    ));


    // Elbow PID Test
    m_driverController.povUp().onTrue(m_arm.setArmPIDCommand(45, 0));
    m_driverController.povRight().onTrue(m_arm.setArmPIDCommand(ArmConstants.IdleAngleSP[0], ArmConstants.IdleAngleSP[1]));
    m_driverController.povDown().onTrue(m_arm.setArmPIDCommand(-45, 0));

    /* Operator Controls */
   
    // Winds up shoot motors then starts intake/feed motor, afterwards stops both motors.
    m_operatorController.rightTrigger().whileTrue(
      m_box.prepareShootCommand()
      .withTimeout(1)
      .andThen(m_box.shootCommand())
      .handleInterrupt(() -> m_box.stopCommand())
    );

    // Intakes note into robot and keeps it there
    m_operatorController.leftTrigger().whileTrue(
      m_box.intakeCommand(false)
      .until(m_box::isForwardLimitSwitchPressed)
    );

    // Reverse intake ( rare use case scenario )
    m_operatorController.back().onTrue(m_box.intakeCommand(true));

    // Arm set point for picking off the floor
    m_operatorController.povDown().onTrue(m_arm.setArmPIDCommand(ArmConstants.FloorAngleSP[0], ArmConstants.FloorAngleSP[1]));

    // Arm set point for picking out of source :)
    m_operatorController.povUp().onTrue(m_arm.setArmPIDCommand(ArmConstants.SourceAngleSP[0], ArmConstants.SourceAngleSP[1]));

    // Arm set point for playing amp
    m_operatorController.a().onTrue(m_arm.setArmPIDCommand(ArmConstants.AmpAngleSP[0], ArmConstants.AmpAngleSP[1]));

    // Arm set point for playing trap
    m_operatorController.x().onTrue(m_arm.setArmPIDCommand(ArmConstants.TrapAngleSP[0], ArmConstants.TrapAngleSP[1]));

    // Arm set point for shooting speaker from the subwoofer
    m_operatorController.y().onTrue(m_arm.setArmPIDCommand(ArmConstants.SpeakerSubwooferAngleSP[0], ArmConstants.SpeakerSubwooferAngleSP[1]));

    // Arm set point for shooting speaker from the podium
    m_operatorController.b().onTrue(m_arm.setArmPIDCommand(ArmConstants.SpeakerPodiumAngleSP[0], ArmConstants.SpeakerPodiumAngleSP[1]));
    
    // Arm set point for shooting horizontally
    m_operatorController.start().onTrue(m_arm.setArmPIDCommand(ArmConstants.HorizontalAngleSP[0], ArmConstants.HorizontalAngleSP[1]));
  
  }

  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }

}
