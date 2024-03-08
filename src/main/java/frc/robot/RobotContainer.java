// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BoxConstants;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.IdleArmCommand;
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
  // Auton chooser
  private SendableChooser<Command> m_autonChooser;

  // Create the driver and operator controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Setup PathPlanner and autons
    m_swerve.setupPathPlanner();
    // PathPlanner named commands
    NamedCommands.registerCommand("ArmToFloor", m_arm.setArmPIDCommand(ArmConstants.ArmState.FLOOR, true).withTimeout(1.5));
    NamedCommands.registerCommand("Intake", m_box.setIntakeMotorCommandThenStop(BoxConstants.kIntakeSpeed).withTimeout(1.75));
    NamedCommands.registerCommand("SpinUpShooter", m_box.setShooterMotorCommand(BoxConstants.kSpeakerShootSpeed));
    NamedCommands.registerCommand("FeedNote", m_box.setIntakeMotorCommand(BoxConstants.kFeedSpeed).withTimeout(0.5));
    NamedCommands.registerCommand("ShootNoteSubwoofer", m_box.ShootNoteSubwoofer().withTimeout(2.25));
    NamedCommands.registerCommand("ShootNoteSubwooferNoRegurgitate", m_box.ShootNoteSubwooferNoRegurgitate().withTimeout(2.5));
    NamedCommands.registerCommand("ArmToIdle", m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, true).withTimeout(1.5) );
    // Allows us to pick our auton in smartdash board
    m_autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton Picker", m_autonChooser);

    // Configure the trigger bindings
    configureBindings();

    // This causes a command scheduler loop overrun
    m_swerve.setDefaultCommand(m_swerve.driveCommandAngularVelocity(
      () -> m_driverController.getLeftY(),
      () -> m_driverController.getLeftX(),
      () -> -m_driverController.getRightX(),
      Constants.OperatorConstants.kFastModeSpeed
    ));

    m_box.setDefaultCommand(m_box.stopCommand());
    //m_arm.setDefaultCommand(new IdleArmCommand(m_arm));
  }


  private void configureBindings() {
    /* Driver Controls */

    // TEST-Vision snapping command.
    // If there is no limelight, there will be an exception
    /*
    m_driverController.rightBumper().whileTrue(
      m_swerve.driveCommandAngularVelocity(
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_limelight.getTargetRotation()/360,
        Constants.OperatorConstants.kFastModeSpeed
      )
    );
    */

    // Changes the wrist angle to what it should be at the moment the button is held.
    // To update the wrist angle, let go and hold the button again
    /*
    m_driverController.rightBumper().whileTrue(
      m_arm.SetWristAngle(getLimelightWristAngle())
    );
    */
    

    // TEST-Drives to and runs a path planner path
    //m_driverController.a().onTrue(m_swerve.driveToPathThenFollowPath(PathPlannerPath.fromPathFile("PlayAmp")));
    
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
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRightX(),
        OperatorConstants.kMidModeSpeed
      )
    );
  
    // Slow speed
    m_driverController.leftTrigger().whileTrue(
      m_swerve.driveCommandAngularVelocity(
        () ->  -m_driverController.getLeftY(),
        () ->  -m_driverController.getLeftX(),
        () -> -m_driverController.getRightX(),
        OperatorConstants.kSlowModeSpeed
      )
    );

    /*
    // Alternate drive mode
    m_driverController.rightBumper().toggleOnTrue(m_swerve.driveCommandPoint(
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX(),
      () -> -m_driverController.getRightX(),
      () -> -m_driverController.getRightY()
    ));
    */

    // Lock the wheels on toggle
    m_driverController.start().toggleOnTrue(
      m_swerve.run(()->{
        m_swerve.lock();
      })
    );

    /* Operator Controls */

    // THIS CODE DOES NOT WORK? I HAVE WORKING CODE ANYWAY
    /*
    m_operatorController.rightTrigger().whileTrue(
      Commands.waitSeconds(4)
      
     //.andThen(m_arm.setArmPIDCommand(ArmSubsystem.getArmState(), true))
     .andThen(m_arm.setArmPIDCommand(ArmConstants.ArmState.AMP, true)) 
      //.alongWith(m_box.setIntakeMotorCommandThenStop(Constants.BoxConstants.kRegurgitateSpeed)
      //.withTimeout(.25)
      //.andThen( m_box.setShooterMotorCommand(ArmSubsystem::getArmState))
      //.withTimeout(m_box.getChargeTime(ArmSubsystem::getArmState))
      //.andThen(m_box.setIntakeMotorCommand(BoxConstants.kFeedSpeed)))
    );
    */

    // Winds up shoot motors then starts intake/feed motor
    m_operatorController.rightTrigger().whileTrue(
      m_box.setShooterIntakeMotorCommand(ArmSubsystem::getArmState)
    ).onFalse(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    // Intakes note into robot and keeps it there
    m_operatorController.leftBumper().whileTrue(m_box.setIntakeMotorCommand(BoxConstants.kIntakeSpeed));

    // Regurgitate Shooter
    m_operatorController.leftTrigger().whileTrue(m_box.setShooterMotorCommand(BoxConstants.kRegurgitateSpeed));

    // Regurgitate Intake
    m_operatorController.rightBumper().whileTrue(m_box.setIntakeMotorCommand(BoxConstants.kRegurgitateSpeed));

    // Idle mode arm set point
    m_operatorController.button(9).whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false));

    // Arm set point for shooting speaker from subwoofer
    m_operatorController.a().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.SHOOT_SUB, true)
    .alongWith(m_box.setShooterMotorCommand(ArmSubsystem::getArmState)));

    // Arm set point for playing amp 
    m_operatorController.x().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.AMP, true)
    .alongWith(m_box.setShooterMotorCommand(ArmSubsystem::getArmState)));

    // Arm set point for shooting speaker from the podium
    m_operatorController.y().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.SHOOT_PODIUM, true)
    .alongWith(m_box.setShooterMotorCommand(ArmSubsystem::getArmState)));

    // Arm set point for shooting trap
    m_operatorController.b().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.TRAP, true)
    .alongWith(m_box.setShooterMotorCommand(ArmSubsystem::getArmState)));

    // Arm set point for shooting horizontal across the field
    m_operatorController.povLeft().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.SHOOT_HORIZONTAL, true)
    .alongWith(m_box.setShooterMotorCommand(ArmSubsystem::getArmState)));
    //m_operatorController.povLeft().whileTrue(m_box.YeetCommand());

    // Arm set point for climbing
    m_operatorController.povRight().whileTrue(m_arm.setArmPIDCommand(ArmConstants.ArmState.CLIMBING_POSITION, false));

    // Manual control toggle for arm
    m_operatorController.start().toggleOnTrue(
        m_arm.manualArmCommand(() -> m_operatorController.getRightY() * 0.3, 
        () -> m_operatorController.getLeftY() * 0.3)
    );

    // Floor intake
    m_operatorController.povDown().whileTrue(
      Commands.sequence(
        Commands.parallel(
          m_arm.setArmPIDCommand(ArmConstants.ArmState.FLOOR, false),
          m_box.setIntakeMotorCommand(BoxConstants.kIntakeSpeed)
        ).until(m_box::isReverseLimitSwitchPressed),
        m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false)
      )
    );

    // Intake from the source
    m_operatorController.povUp().whileTrue(
      Commands.sequence(
        Commands.parallel(
          m_arm.setArmPIDCommand(ArmConstants.ArmState.SOURCE, false),
          m_box.setIntakeMotorCommand(BoxConstants.kIntakeSpeed)
        ).until(m_box::isReverseLimitSwitchPressed),
        m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE, false)
      )
    );

    // Reset wrist encoder
    m_operatorController.back().onTrue(Commands.runOnce(() -> m_arm.resetWristEncoder()));
  }

  // AutonomousCommand
  public Command getAutonomousCommand() {
    return m_autonChooser.getSelected();
  }
}