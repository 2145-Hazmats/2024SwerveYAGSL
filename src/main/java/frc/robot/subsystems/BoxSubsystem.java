// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Constants.BoxConstants;
import frc.robot.Constants.ArmConstants.ArmPosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BoxSubsystem extends SubsystemBase {
  // Declare and intialize motor variables to a new instance of CANSparkMax
  private CANSparkMax shooterMotor = new CANSparkMax(BoxConstants.kShooterMotorID, MotorType.kBrushless);
  private CANSparkMax intakeMotor = new CANSparkMax(BoxConstants.kIntakeMotorID, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();


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

  /**
   * Sets the speed of the intake motor. Takes a double for speed and a boolean for reversed.
   *
   * @param speed     Speed of the motor.
   * @param reversed  False = intakes or shoots the note. True = regurgitates the note.
   */
  public Command setIntakeMotorCommand(double speed) {
    return run(() -> intakeMotor.set(speed));
  }

  /**
   * Sets the speed of the shooter motor.
   *
   * @param speed     Speed of the motor.
   */
  public Command setShooterMotorCommand(double speed) {
    return run(() -> shooterMotor.set(speed));
  }

  public Command setShooterMotorCommand(ArmPosition position) {
    switch(position) {
      case SHOOT_SUB:
        return run(() -> shooterMotor.set(BoxConstants.kSpeakerShootSpeed));
      case AMP:
        return run(() -> shooterMotor.set(BoxConstants.kAmpShootSpeed));
      default:
        return run(() -> shooterMotor.set(BoxConstants.kDeafultShootSpeed));
    }
  }

  /**
   * Stops the intake and shooter motor.
   */
  public Command stopCommand() {
    return runOnce( () -> {
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
  }

}