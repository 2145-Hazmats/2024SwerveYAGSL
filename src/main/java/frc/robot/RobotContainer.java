// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BoxConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BoxSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


// This class is instantiated when the robot is first started up
public class RobotContainer {
  // Make the one and only instance of each subsystem
  private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File (Filesystem.getDeployDirectory(), "swerve"));
  private final BoxSubsystem m_box = new BoxSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  // Other variables
  private SendableChooser<Command> m_autonChooser;

  // Create the driver and operator controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Setup PathPlanner and put autons on SmartDashboard
    m_swerve.setupPathPlanner();
    m_autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Picker", m_autonChooser);

    // PathPlanner named commands
    NamedCommands.registerCommand("ArmToFloor", m_arm.setArmPIDCommand(ArmConstants.kFloorAngleSP[0], ArmConstants.kFloorAngleSP[1]).withTimeout(1.5));
    NamedCommands.registerCommand("Intake", m_box.setIntakeSpeedCommand(BoxConstants.kIntakeSpeed).until(m_box::isReverseLimitSwitchPressed));
    NamedCommands.registerCommand("SpinUpShooter", m_box.setShooterSpeedCommand(BoxConstants.kShooterSpeed));
    NamedCommands.registerCommand("ShootNoteSubwoofer", m_box.setIntakeSpeedCommand(BoxConstants.kFeedSpeed).withTimeout(0.5));
  
    m_swerve.setDefaultCommand(m_swerve.driveCommandAngularVelocity(
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRawAxis(4),
        Constants.OperatorConstants.kFastModeSpeed
    ));

    m_box.setDefaultCommand(m_box.stopCommand());

    m_arm.setDefaultCommand(m_arm.manualArmCommand(
      () -> m_operatorController.getRightY() * 0.3, 
      () -> m_operatorController.getLeftY() * 0.3)
    );
  }


  private void configureBindings() {
    /* Driver Controls */

    // Lock the wheels on toggle
    m_driverController.start().toggleOnTrue(
      m_swerve.run(()->{
        m_swerve.lock();
      })
    );

    // Rotate towards the driver
    m_driverController.a().whileTrue(m_swerve.driveCommandPoint(() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
      () -> 0,
      () -> -1
    ));

    // Rotate to the right
    m_driverController.b().whileTrue(m_swerve.driveCommandPoint(() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
      () -> 1,
      () -> 0
    ));

    // Rotate to the left
    m_driverController.x().whileTrue(m_swerve.driveCommandPoint(() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
      () -> -1,
      () -> 0
    ));

    // Rotate away from the driver
    m_driverController.y().whileTrue(m_swerve.driveCommandPoint(() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
      () -> 0,
      () -> 1
    ));

    // Resets the gyro
    m_driverController.back().onTrue(
      m_swerve.runOnce(()->{
        m_swerve.resetGyro();
      })
    );
 
    // Medium speed
    m_driverController.rightTrigger().whileTrue(
      m_swerve.driveCommandAngularVelocity(
        () -> -m_driverController.getLeftY(),
        () ->-m_driverController.getLeftX(),
        () -> -m_driverController.getRawAxis(4),
        OperatorConstants.kMidModeSpeed
      )
    );
  
    // Slow speed
    m_driverController.leftTrigger().whileTrue(
      m_swerve.driveCommandAngularVelocity(
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRawAxis(4),
        OperatorConstants.kSlowModeSpeed
      )
    );

    // Alternate drive mode
    m_driverController.rightBumper().toggleOnTrue(m_swerve.driveCommandPoint(
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX(),
      () -> -m_driverController.getRightX(),
      () -> -m_driverController.getRightY()
    ));

    // Elbow PID Test
    m_operatorController.povUp().onTrue(m_arm.setArmPIDCommand(-90, 0));
    m_operatorController.povRight().onTrue(m_arm.setArmPIDCommand(ArmConstants.kIdleAngleSP[0], ArmConstants.kIdleAngleSP[1]));
    m_operatorController.povDown().onTrue(m_arm.setArmPIDCommand(-45, 0));

    /* Operator Controls */

    // Winds up shoot motors then starts intake/feed motor, afterwards stops both motors.
    /*
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
    */

    // Winds up shoot motors then starts intake/feed motor
    m_operatorController.rightTrigger().whileTrue(
      m_box.setShooterSpeedCommand(BoxConstants.kShooterSpeed)
      .withTimeout(BoxConstants.kShooterDelay)
      .andThen(m_box.setIntakeSpeedCommand(BoxConstants.kFeedSpeed))
    );

    // Intakes note into robot and keeps it there
    m_operatorController.leftTrigger().whileTrue(
      m_box.setIntakeSpeedCommand(BoxConstants.kIntakeSpeed)
      .until(m_box::isReverseLimitSwitchPressed)
    );

    // Regurgitate
    m_operatorController.leftBumper().whileTrue(
      m_box.setIntakeSpeedCommand(BoxConstants.kRegurgitateSpeed)
    );

    // Stop the box motors
    m_operatorController.rightBumper().whileTrue(
      m_box.stopCommand()
    );

    // Manual control toggle for arm and wrist
    // UPDATE: Made the Arm's default command manual control in the RobotContainer constructor for Oxford
    /*
    m_operatorController.button(9).toggleOnTrue(m_arm.manualElbowCommand(m_operatorController.getLeftY()));
    m_operatorController.button(10).toggleOnTrue(m_arm.manualWristCommand(m_operatorController.getRightY()));
    */

    // Arm set point for picking off the floor
    m_operatorController.povDown().onTrue(m_arm.setArmPIDCommand(ArmConstants.kFloorAngleSP[0], ArmConstants.kFloorAngleSP[1]));

    // Arm set point for picking out of source
    m_operatorController.povUp().onTrue(m_arm.setArmPIDCommand(ArmConstants.kSourceAngleSP[0], ArmConstants.kSourceAngleSP[1]));

    // Arm set point for playing amp
    m_operatorController.a().onTrue(m_arm.setArmPIDCommand(ArmConstants.kAmpAngleSP[0], ArmConstants.kAmpAngleSP[1]));

    // Arm set point for playing trap
    m_operatorController.x().onTrue(m_arm.setArmPIDCommand(ArmConstants.kTrapAngleSP[0], ArmConstants.kTrapAngleSP[1]));

    // Arm set point for shooting speaker from the subwoofer
    m_operatorController.y().onTrue(m_arm.setArmPIDCommand(ArmConstants.kSpeakerSubwooferAngleSP[0], ArmConstants.kSpeakerSubwooferAngleSP[1]));

    // Arm set point for shooting speaker from the podium
    m_operatorController.b().onTrue(m_arm.setArmPIDCommand(ArmConstants.kSpeakerPodiumAngleSP[0], ArmConstants.kSpeakerPodiumAngleSP[1]));
    
    // Arm set point for shooting horizontally
    m_operatorController.povRight().onTrue(m_arm.setArmPIDCommand(ArmConstants.kHorizontalAngleSP[0], ArmConstants.kHorizontalAngleSP[1]));
  }

  // AutonomousCommand
  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }

}
