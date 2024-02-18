// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Constants.BoxConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class BoxSubsystem extends SubsystemBase {
  // Declare and intialize motor variables to a new instance of CANSparkMax
  private CANSparkMax shooterMotor = new CANSparkMax(BoxConstants.kShooterMotorID, MotorType.kBrushless);
  private CANSparkMax intakeMotor = new CANSparkMax(BoxConstants.kIntakeMotorID, MotorType.kBrushless);


  /** Creates a new Box. */
  public BoxSubsystem() {
    // Restore factory defaults
    intakeMotor.restoreFactoryDefaults();
    shooterMotor.restoreFactoryDefaults();
    // Enable voltage compensation
    intakeMotor.enableVoltageCompensation(BoxConstants.kIntakeMotorNominalVoltage);
    shooterMotor.enableVoltageCompensation(BoxConstants.kShooterMotorNominalVoltage);
    // Invert the shooter motor
    shooterMotor.setInverted(true);
  }

  // This commands sorta work
  /*
  public Command intakeCommand(boolean reversed) {
    return startEnd(
      () -> intakeMotor.set(reversed ? -BoxConstants.kIntakeSpeed : BoxConstants.kIntakeSpeed),
      () -> intakeMotor.set(0)
    );
  }


  public Command shootCommand() {
    return runOnce(() -> shooterMotor.set(BoxConstants.kShooterSpeed));
  }


  public Command prepareShootCommand() {
    return startEnd(
      () -> intakeMotor.set(BoxConstants.kFeedSpeed), 
      () -> stopCommand()
    );
  }
  */

  /**
   * Sets the speed of the intake motor. Takes a double for speed and a boolean for reversed.
   *
   * @param speed     Speed of the motor.
   * @param reversed  False = intakes or shoots the note. True = regurgitates the note.
   */
  public Command setIntakeSpeedCommand(double speed) {
    return run(() -> intakeMotor.set(speed));
  }

  public Command shootCommand(double intakeSpeed, double shooterSpeed) {
    return new ParallelCommandGroup(
      startEnd(
      () -> setShooterSpeedCommand(shooterSpeed), 
      () -> setShooterSpeedCommand(0)
      ),
      new SequentialCommandGroup( 
        new WaitCommand(1),
        startEnd(
          () -> setIntakeSpeedCommand(intakeSpeed), 
          () -> setIntakeSpeedCommand(0))
      )
      );


  }
  /**
   * Sets the speed of the shooter motor.
   *
   * @param speed     Speed of the motor.
   */
  public Command setShooterSpeedCommand(double speed) {
    return run(() -> shooterMotor.set(speed));
  }

  /**
   * Stops the intake and shooter motor.
   */
  public Command stopCommand() {
    return runOnce( () -> {
      intakeMotor.set(0);
      shooterMotor.set(0);
    });
  }

  /**
   * Returns a boolean based on if the forward limit switch is pressed.
   */
  public boolean isReverseLimitSwitchPressed() {
    return intakeMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("isReverseLimitSwitchPressed", isReverseLimitSwitchPressed());
  }

}