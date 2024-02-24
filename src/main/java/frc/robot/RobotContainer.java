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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BoxConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IdleArmCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BoxSubsystem;
//import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;



// This class is instantiated when the robot is first started up
public class RobotContainer {
  // Make the one and only object of each subsystem
  private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File (Filesystem.getDeployDirectory(), "swerve"));
  private final BoxSubsystem m_box = new BoxSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  //private final LimelightSubsystem m_limelight = new LimelightSubsystem(m_swerve);
  
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

    // Setup PathPlanner and autons
    m_swerve.setupPathPlanner();
    // PathPlanner named commands
    NamedCommands.registerCommand("ArmToFloor", m_arm.setArmPIDCommandAndStay(ArmConstants.ArmState.FLOOR).withTimeout(1.5));
    NamedCommands.registerCommand("Intake", m_box.setIntakeMotorCommandThenStop(BoxConstants.kIntakeSpeed).withTimeout(.75));
    NamedCommands.registerCommand("SpinUpShooter", m_box.setShooterMotorCommand(BoxConstants.kSpeakerShootSpeed));
    NamedCommands.registerCommand("FeedNote", m_box.setIntakeMotorCommand(BoxConstants.kFeedSpeed).withTimeout(0.5));
    NamedCommands.registerCommand("ShootNoteSubwoofer",
    //NamedCommands.registerCommand("ShootNoteSubwoofer", m_box.shootCommand(m_box.setShooterMotorCommand(() -> m_arm.getArmState())
     m_box.ShootPieceAtSubwoofer()
      );

    m_autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Picker", m_autonChooser);

    // This causes a command scheduler loop overrun
    m_swerve.setDefaultCommand(m_swerve.driveCommandAngularVelocity(
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRawAxis(4),
        Constants.OperatorConstants.kFastModeSpeed
    ));

    m_box.setDefaultCommand(m_box.stopCommand());
    //m_arm.setDefaultCommand(new IdleArmCommand(m_arm));
  }


  private void configureBindings() {
    /* Driver Controls */

    // Vision snapping command. Active when the right axis is 0
    // If there is no limelight, there will be an exception
    /*m_driverController.axisGreaterThan(4, OperatorConstants.kVisionModeDeadband)
    .or(m_driverController.axisLessThan(4, -OperatorConstants.kVisionModeDeadband)).whileFalse(
      m_swerve.driveCommandAngularVelocity(
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_limelight.getTargetRotation()/360,
        Constants.OperatorConstants.kFastModeSpeed
      )
    );*/

    // These commands work, but the drivers weren't using them. I'm commenting them out so we can reuse these buttons for other commands
    /*
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
    */

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

    // Lock the wheels on toggle
    m_driverController.start().toggleOnTrue(
      m_swerve.run(()->{
        m_swerve.lock();
      })
    );

    /* Operator Controls */

    // Winds up shoot motors then starts intake/feed motor
    m_operatorController.rightTrigger().whileTrue(
      m_box.setShooterMotorCommand(() -> m_arm.getArmState())
      //m_box.setShooterMotorCommand(BoxConstants.kDeafultShootSpeed)
      .withTimeout(BoxConstants.kShooterDelay)
      .andThen(m_box.setIntakeMotorCommand(BoxConstants.kFeedSpeed))
      //m_box.setShooterMotorCommand(.25).withTimeout(2).andThen(m_box.setIntakeMotorCommand(.25))
    );

    // Intakes note into robot and keeps it there
    m_operatorController.leftTrigger().whileTrue(
      m_box.setIntakeMotorCommand(BoxConstants.kIntakeSpeed)
      .until(m_box::isReverseLimitSwitchPressed)
    );

    // Regurgitate
    m_operatorController.leftBumper().whileTrue(
      m_box.setIntakeMotorCommand(BoxConstants.kRegurgitateSpeed)
    );

    m_operatorController.a().whileTrue(
      new IntakeCommand(m_arm, m_box)
    );

    /* //SOURCE COMMAND UNCOMMENT OUT LATER AND MAP A BUTTON
    m_operatorController.b().whileTrue(
      new IntakeCommand(m_arm, m_box, ArmConstants.kFloorAngleSP[0], ArmConstants.kFloorAngleSP[1])
    );
    */

    // Manual control toggle for arm and wrist
    // UPDATE: Made the Arm's default command manual control in the RobotContainer constructor for Oxford
    /*
    m_operatorController.button(9).toggleOnTrue(m_arm.manualElbowCommand(m_operatorController.getLeftY()));
    m_operatorController.button(10).toggleOnTrue(m_arm.manualWristCommand(m_operatorController.getRightY()));
    */

    // Arm set point for picking off the floor
    m_operatorController.povDown().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.FLOOR));

    // Arm set point for picking out of source
    m_operatorController.povUp().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.SOURCE));

    // Arm set point for playing amp
    //m_operatorController.a().onTrue(m_arm.setArmPIDCommand(ArmConstants.kAmpAngleSP[0], ArmConstants.kAmpAngleSP[1]));
    m_operatorController.start().toggleOnTrue(
        m_arm.manualArmCommand(() -> m_operatorController.getRightY() * 0.3, 
        () -> m_operatorController.getLeftY() * 0.3)
    );

    m_operatorController.back().onTrue(Commands.runOnce(() -> m_arm.resetWrist()));

    // Arm set point for playing trap
    m_operatorController.x().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.AMP));

    // Arm set point for shooting speaker from the subwoofer
    m_operatorController.y().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.SHOOT_SUB));

    // Arm set point for shooting speaker from the podium
    //m_operatorController.b().whileTrue(m_arm.setArmPIDCommand(ArmConstants.kSpeakerPodiumAngleSP[0], ArmConstants.kSpeakerPodiumAngleSP[1]));
    
    // Arm set point for shooting horizontally
    m_operatorController.povRight().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE).withTimeout(2).andThen(m_arm.PIDFallin()).andThen(() -> m_arm.resetWrist())); 

    // some code button thing ask riley idk
    m_operatorController.povLeft().whileTrue(
      Commands.sequence(
        Commands.parallel(
          m_arm.setArmPIDCommand(ArmConstants.ArmState.FLOOR),
          m_box.setIntakeMotorCommand(BoxConstants.kIntakeSpeed)
        ).until(m_box::isReverseLimitSwitchPressed),
        m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE)
      )
    );
  }

  // AutonomousCommand
  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }

}