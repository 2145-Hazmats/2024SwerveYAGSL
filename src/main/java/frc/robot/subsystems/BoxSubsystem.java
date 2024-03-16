// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import frc.robot.Constants;
import frc.robot.Constants.BoxConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BoxSubsystem extends SubsystemBase {
  // Declare and intialize motor variables to a new instance of CANSparkMax
  private final CANSparkMax topShooterMotor = new CANSparkMax(BoxConstants.kTopShooterMotorID, MotorType.kBrushless);
  private final RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
  private final CANSparkMax bottomShooterMotor = new CANSparkMax(BoxConstants.kBottomShooterMotorID, MotorType.kBrushless);
  private final RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();
  private final CANSparkMax intakeMotor = new CANSparkMax(BoxConstants.kIntakeMotorID, MotorType.kBrushless);
  // shooterMotor variables
  private double topShooterSpeed = 0.0; 
  private double bottomShooterSpeed = 0.0;
  // Sensor 
  private static DigitalInput noteSensor = new DigitalInput(BoxConstants.kNoteSensorChannel);

  /** Creates a new Box. */
  public BoxSubsystem() {
    /* Motor Configuration */

    // Restore factory defaults of the Spark Max.
    // It's important to have all Spark Maxs behave the expected way, especially if we switch to a different Spark Max in the middle of a competition.
    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    // Set motor current limit
    topShooterMotor.setSmartCurrentLimit(20);
    bottomShooterMotor.setSmartCurrentLimit(40);
    intakeMotor.setSmartCurrentLimit(20);

    // Enable voltage compensation
    intakeMotor.enableVoltageCompensation(BoxConstants.kIntakeMotorNominalVoltage);
    topShooterMotor.enableVoltageCompensation(BoxConstants.kShooterMotorNominalVoltage);
    bottomShooterMotor.enableVoltageCompensation(BoxConstants.kShooterMotorNominalVoltage);

    // Reduce the frequency of the motor position sent to the roboRIO
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    topShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    bottomShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    // Set the idle mode of the motors
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    // Invert the shooter motor
    topShooterMotor.setInverted(true);
    bottomShooterMotor.setInverted(true);

    /* Encoder Configuration */

    // Setup encoder conversion factors
    topShooterEncoder.setPositionConversionFactor(1);
    bottomShooterEncoder.setPositionConversionFactor(1);

    // Set encoders position to 0
    topShooterEncoder.setPosition(0);
    bottomShooterEncoder.setPosition(0);
  }
  

  /**
   * Sets the speed of the intake motor. Takes a double for speed.
   *
   * @param speed     Speed of the motor.   */
  public Command setIntakeMotorCommand(double speed) {
    return run(() -> intakeMotor.set(speed));
  }


  public Command setIntakeMotorCommandThenStop(double speed) {
    return Commands.startEnd(() -> intakeMotor.set(speed), () -> intakeMotor.set(0), this);
  }


  public void Yeet() {
    intakeMotor.set(Constants.BoxConstants.kYeetSpeedIntake);
    topShooterMotor.set(Constants.BoxConstants.kTopYeetSpeed);
    bottomShooterMotor.set(Constants.BoxConstants.kBottomYeetSpeed);
  }

  
  public Command YeetCommand() {
    return Commands.startEnd(() -> Yeet(), () -> Yeet(), this);
  }


  public Command ShootNoteSubwoofer() {
    return setIntakeMotorCommandThenStop(Constants.BoxConstants.kRegurgitateSpeed)
    .withTimeout(.25) 
    .andThen(setShooterMotorCommand(Constants.BoxConstants.kTopSpeakerSpeed))
    .withTimeout(BoxConstants.kShooterDelay)
    .andThen(setIntakeMotorCommandThenStop(BoxConstants.kFeedSpeed))
    .withTimeout(2.0)
    .andThen(setShooterMotorCommand(0));
  }


  public Command ShootNoteSubwooferNoRegurgitate() {
    return setShooterMotorCommand(0.34)
    .withTimeout(BoxConstants.kShooterDelay)
    .andThen(setIntakeMotorCommandThenStop(BoxConstants.kFeedSpeed))
    .withTimeout(2.0)
    .andThen(setShooterMotorCommand(0));
  }


  /**
   * Sets the speed of the shooter motor.
   *
   * @param speed     Speed of the motor.
   */
  public Command setShooterMotorCommand(double speed) {
    return run(() -> {
      topShooterMotor.set(speed);
      bottomShooterMotor.set(speed);
    });
  }

  // DELETE THIS
  /*
  public Command setShooterMotorCommandThenStop(double speed) {
    return Commands.startEnd(() -> {
      topShooterMotor.set(speed);
      bottomShooterMotor.set(speed);
    }, () -> {
      topShooterMotor.set(0);
      bottomShooterMotor.set(0);
    }, this);
  }
  */


  /**
   * Sets the speed of the shooter motor depending on a supplied ArmState.
   *
   * @param position  ArmState Supplier used to set the speed of the shooter motor.
   * @param feeder    A boolean if the feeder motor should also spin. 
   *                  True = feeder motor spins. False = feeder motor does NOT spin.
   */
  public Command setShooterFeederCommand(Supplier<ArmState> position, boolean feeder) {
    return run(() -> {
      switch(position.get()) {
        case SHOOT_SUB:
          topShooterSpeed = BoxConstants.kTopSpeakerSpeed;
          bottomShooterSpeed = BoxConstants.kBottomSpeakerSpeed;
          break;
        case AMP:
          topShooterSpeed = BoxConstants.kTopAmpSpeed;
          bottomShooterSpeed = BoxConstants.kBottomAmpSpeed;
          break;
        case IDLE:
          topShooterSpeed = 0.0;
          bottomShooterSpeed = 0.0;
          break;
        case SHOOT_HORIZONTAL:
          topShooterSpeed = BoxConstants.kTopHorizontalSpeed;
          bottomShooterSpeed = BoxConstants.kBottomHorizontalSpeed;
          break;
        default:
          topShooterSpeed = BoxConstants.kTopDefaultSpeed;
          bottomShooterSpeed = BoxConstants.kBottomDefaultSpeed;
          break;
      }
      topShooterMotor.set(topShooterSpeed);
      bottomShooterMotor.set(bottomShooterSpeed);

      if (feeder) {
        intakeMotor.set(BoxConstants.kFeedSpeed);
      }

    });
  }


  /**
   * Stops the intake and shooter motor.
   */
  public Command stopCommand() {
    return runOnce(() -> {
      topShooterMotor.stopMotor();
      bottomShooterMotor.stopMotor();
      intakeMotor.stopMotor();
    });
  }


  public static boolean noteSensorTriggered() {
    return noteSensor.get();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("topShooterMotor Velocity", topShooterEncoder.getVelocity());
    SmartDashboard.putNumber("bottomShooterMotor Velocity", bottomShooterEncoder.getVelocity());
  }

}