// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Constants.BoxConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Constants.ArmConstants.ArmState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class BoxSubsystem extends SubsystemBase {
  // Declare and intialize motor variables to a new instance of CANSparkMax
  private CANSparkMax shooterMotor = new CANSparkMax(BoxConstants.kShooterMotorID, MotorType.kBrushless);
  private CANSparkMax intakeMotor = new CANSparkMax(BoxConstants.kIntakeMotorID, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

  private double shooterMotorSpeed = 0.0;


  /** Creates a new Box. */
  public BoxSubsystem() {
    // Restore factory defaults
    //intakeMotor.restoreFactoryDefaults();
    //shooterMotor.restoreFactoryDefaults();
    // Enable voltage compensation
    intakeMotor.enableVoltageCompensation(BoxConstants.kIntakeMotorNominalVoltage);
    shooterMotor.enableVoltageCompensation(BoxConstants.kShooterMotorNominalVoltage);
    // Invert the shooter motor
    shooterMotor.setInverted(true);
  }

  /**
   * Sets the speed of the intake motor. Takes a double for speed and a boolean for reversed.
   *
   * @param speed     Speed of the motor.
   * @param reversed  False = intakes or shoots the note. True = regurgitates the note.
   */
  public Command setIntakeMotorCommand(double speed) {
    return run(() -> intakeMotor.set(speed));
  }

  public Command setIntakeMotorCommandAuto(double speed){
    return runOnce(() -> intakeMotor.set(speed));
  }

  public Command setIntakeMotorCommandThenStop(double speed) {
    return Commands.startEnd(() -> intakeMotor.set(speed), () -> intakeMotor.set(0), this);

  }
  public Command ShootPieceAtSubwoofer() {
    return setShooterMotorCommandAuto(BoxConstants.kSpeakerShootSpeed)
    .andThen(new WaitCommand(2.5))
    .andThen(setIntakeMotorCommandAuto(BoxConstants.kFeedSpeed))
    .andThen(new WaitCommand(0.75))
    .andThen(stopCommand());
  }

  /**
   * Sets the speed of the shooter motor.
   *
   * @param speed     Speed of the motor.
   */
  public Command setShooterMotorCommand(double speed) {
    return run(() -> shooterMotor.set(speed));
  }

  public Command setShooterMotorCommandAuto(double speed) {
    return runOnce(() -> shooterMotor.set(speed));
  }

  public Command setShooterMotorCommandThenStop(double speed) {
    return Commands.startEnd(() -> shooterMotor.set(speed),() -> shooterMotor.set(0),this);
  }

  public Command setShooterMotorCommand(Supplier<ArmState> position) {
    return run(() -> {

      SmartDashboard.putString("Shooter Motor Command Position", position.get().toString());

      switch(position.get()) {
        case SHOOT_SUB:
          shooterMotorSpeed = BoxConstants.kSpeakerShootSpeed;
          break;
        case AMP:
          shooterMotorSpeed = BoxConstants.kAmpShootSpeed;
          break;
        case IDLE:
          shooterMotorSpeed = 0.0;
          break;
        default:
          shooterMotorSpeed = BoxConstants.kDefaultShootSpeed;
          break;
      }

      shooterMotor.set(shooterMotorSpeed);
    });
  }

  /**
   * Stops the intake and shooter motor.
   */
  public Command stopCommand() {
    return runOnce(() -> {
      intakeMotor.stopMotor();
      shooterMotor.stopMotor();
    });
  }


  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
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
    SmartDashboard.putNumber("Shooter Motor Speed", shooterMotorSpeed);
  }

}