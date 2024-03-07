// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Constants;
import frc.robot.Constants.BoxConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BoxSubsystem extends SubsystemBase {
  // Declare and intialize motor variables to a new instance of CANSparkMax
  private CANSparkMax shooterMotor = new CANSparkMax(BoxConstants.kShooterMotorID, MotorType.kBrushless);
  private CANSparkMax theOtherShooterMotor = new CANSparkMax(BoxConstants.kTheOtherShooterMotorID, MotorType.kBrushless);
  private CANSparkMax intakeMotor = new CANSparkMax(BoxConstants.kIntakeMotorID, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  // Speed of the shooter motor
  private double shooterMotorSpeed = 0.0;
  private double shooterChargeTime = Constants.BoxConstants.kShooterDelay;

  /** Creates a new Box. */
  public BoxSubsystem() {
    /* Motor Configuration */

    // Restore factory defaults of the Spark Max.
    // It's important to have all Spark Maxs behave the expected way, especially if we switch to a different Spark Max in the middle of a competition.
    shooterMotor.restoreFactoryDefaults();
    theOtherShooterMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    // Set motor current limit
    shooterMotor.setSmartCurrentLimit(40);
    theOtherShooterMotor.setSmartCurrentLimit(40);
    intakeMotor.setSmartCurrentLimit(40);

    // Enable voltage compensation
    intakeMotor.enableVoltageCompensation(BoxConstants.kIntakeMotorNominalVoltage);
    shooterMotor.enableVoltageCompensation(BoxConstants.kShooterMotorNominalVoltage);
    theOtherShooterMotor.enableVoltageCompensation(BoxConstants.kShooterMotorNominalVoltage);

    // Set the idle mode of the motors
    shooterMotor.setIdleMode(IdleMode.kCoast);
    theOtherShooterMotor.setIdleMode(IdleMode.kCoast);
    // HELP! Should the intake motor be in coast or brake mode?
    //intakeMotor.setIdleMode(IdleMode.kCoast);

    // Invert the shooter motor
    shooterMotor.setInverted(true);

    // Have the second shooter motor follow the first
    theOtherShooterMotor.follow(shooterMotor);

    /* Encoder Configuration */

    /* PIDControllers */

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


  public Command ShootNoteAmp() {
    return setShooterMotorCommand(ArmSubsystem::getArmState)
    .withTimeout(getChargeTime(ArmSubsystem::getArmState))
    .andThen(setIntakeMotorCommand(BoxConstants.kFeedSpeed))
    .withTimeout(2.0);
  }


  public Command ShootNoteSubwoofer() {
    return setIntakeMotorCommandThenStop(Constants.BoxConstants.kRegurgitateSpeed)
    .withTimeout(.25) 
    .andThen(setShooterMotorCommand(Constants.BoxConstants.kSpeakerShootSpeed))
    .withTimeout(getChargeTime(ArmSubsystem::getArmState))
    .andThen(setIntakeMotorCommandThenStop(BoxConstants.kFeedSpeed))
    .withTimeout(2.0)
    .andThen(setShooterMotorCommand(0));
  }


  public Command ShootNoteSubwooferNoRegurgitate() {
    return //setIntakeMotorCommandThenStop(Constants.BoxConstants.kRegurgitateSpeed)
    //.withTimeout(.25) 
    setShooterMotorCommand(0.34)
    .withTimeout(getChargeTime(ArmSubsystem::getArmState))
    .andThen(setIntakeMotorCommandThenStop(BoxConstants.kFeedSpeed))
    .withTimeout(2.0)
    .andThen(setShooterMotorCommand(0));
  }


  public Command ShootNoteAuton() {
    return setShooterMotorCommand(ArmSubsystem::getArmState)
    .withTimeout(getChargeTime(ArmSubsystem::getArmState))
    .andThen(setIntakeMotorCommand(BoxConstants.kFeedSpeed))
    .withTimeout(2.0);
  }


  /**
   * Sets the speed of the shooter motor.
   *
   * @param speed     Speed of the motor.
   */
  public Command setShooterMotorCommand(double speed) {
    return run(() -> shooterMotor.set(speed));
  }


  public Command setTheOtherShooterMotorCommand(double speed) {
    return run(() -> theOtherShooterMotor.set(speed));
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
        case SHOOT_HORIZONTAL:
          shooterMotorSpeed = BoxConstants.kHorizontalShootSpeed;
          break;
        default:
          shooterMotorSpeed = BoxConstants.kDefaultShootSpeed;
          break;
      }
      shooterMotor.set(shooterMotorSpeed);
    });
  }


  // This is broken it always returns the intialized shooterChargeTime value
  public double getChargeTime(Supplier<ArmState> position) {
    switch(position.get()) {
        case AMP:
          shooterChargeTime = BoxConstants.kShooteDelayAmp;
          break;
        
        default:
          shooterMotorSpeed = BoxConstants.kShooterDelay;
          break;
      }
    return shooterChargeTime;
  }


  /**
   * Stops the intake and shooter motor.
   */
  public Command stopCommand() {
    return runOnce(() -> {
      intakeMotor.stopMotor();
      shooterMotor.stopMotor();
      theOtherShooterMotor.stopMotor();
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