// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;
import frc.robot.Constants.BoxConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  // Get the PIDController object for the 2 shooter motors
  private SparkPIDController topPIDController = topShooterMotor.getPIDController();
  private SparkPIDController bottomPIDController = bottomShooterMotor.getPIDController();
  // SimpleMotorFeedforward for the 2 shooter motors
  // TODO: USE THESE VALUES IN SimpleMotorFeedforward
  private SimpleMotorFeedforward topFeedForward = new SimpleMotorFeedforward(
      BoxConstants.kTopS,
      BoxConstants.kTopV);
  private SimpleMotorFeedforward bottMotorFeedforward = new SimpleMotorFeedforward(
      BoxConstants.kBottomS,
      BoxConstants.kBottomV);

  // Variables used during SmartDashboard changes
  private double topP, topFF, topSetPoint = 0;
  private double bottomP, bottomFF, bottomSetPoint = 0;
  // shooterMotor variables
  private double shooterMotorSpeed = 0.0;
  private double shooterChargeTime = Constants.BoxConstants.kShooterDelay;
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
    bottomShooterMotor.setSmartCurrentLimit(20);
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

    // Have the second shooter motor follow the first
    //bottomShooterMotor.follow(topShooterMotor);

    /* Encoder Configuration */

    // Setup encoder conversion factors
    topShooterEncoder.setPositionConversionFactor(1);
    bottomShooterEncoder.setPositionConversionFactor(1);

    // Set encoders position to 0
    topShooterEncoder.setPosition(0);
    bottomShooterEncoder.setPosition(0);

    /* PIDControllers */

    // Set PIDController FeedbackDevice
    //topPIDController.setFeedbackDevice(topShooterEncoder);
    //bottomPIDController.setFeedbackDevice(bottomShooterEncoder);

    // Setup the shooterMotor PIDControllers
    // Constant RPM tuning on a flywheel that doesn't change RPM setpoints dynamic only needs a P.
    // The I is not needed because of FF (and we avoid it because of integral windup), and the D does nothing.
    // Feedforward only needs a kS and kV, so we will use WPILib's SimpleMotorFeedforward
    topPIDController.setP(ArmConstants.kElbowP);
    bottomPIDController.setP(ArmConstants.kWristP);

    // Put shooterMotor PIDs on SmartDashboard  
    SmartDashboard.putNumber("topShooter P", BoxConstants.kTopShooterP);
    SmartDashboard.putNumber("topShooter Set Point", 0); 
    SmartDashboard.putNumber("bottomShooter P", BoxConstants.kBottomShooterP);
    SmartDashboard.putNumber("bottomShooter Set Point", 0);
  }
  

  /**
   * Sets the speed of the intake motor. Takes a double for speed.
   *
   * @param speed     Speed of the motor.   */
  public Command setIntakeMotorCommand(double speed) {
    return run(() -> intakeMotor.set(speed));
  }


  public Command setIntakeMotorCommandAuto(double speed){
    return runOnce(() -> intakeMotor.set(speed));
  }


  public Command setIntakeMotorCommandThenStop(double speed) {
    return Commands.startEnd(() -> intakeMotor.set(speed), () -> intakeMotor.set(0), this);
  }


  public void Yeet() {
    intakeMotor.set(Constants.BoxConstants.kYeetSpeedIntake);
    topShooterMotor.set(Constants.BoxConstants.kYeetSpeedShooter);
    bottomShooterMotor.set(Constants.BoxConstants.kYeetSpeedShooter);
  }

  
  public Command YeetCommand() {
    return Commands.startEnd(() -> Yeet(), () -> Yeet(), this);
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


  /**
   * Sets the speed of the shooter motor.
   *
   * @param speed     Speed of the motor.
   */
  // DOES THIS HAVE TO BE RUN? WHY NOT runOnce()
  public Command setShooterMotorCommand(double speed) {
    return run(() -> {
      topShooterMotor.set(speed);
      bottomShooterMotor.set(speed);
    });
  }


  public Command setShooterMotorCommandThenStop(double speed) {
    return Commands.startEnd(() -> {
      topShooterMotor.set(speed);
      bottomShooterMotor.set(speed);
    }, () -> {
      topShooterMotor.set(0);
      bottomShooterMotor.set(0);
    }, this);
  }


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
      topShooterMotor.set(shooterMotorSpeed);
      bottomShooterMotor.set(shooterMotorSpeed);

      if (feeder) {
        intakeMotor.set(BoxConstants.kFeedSpeed);
      }

    });
  }


  // This might be broken? it might always returns the intialized shooterChargeTime value, we have to check
  // THIS METHOD IS ONLY USED FOR SHOOTING THE SUBWOOFER IN AUTON. SO THIS METHOD SEEMS USELESS.
  // USE A CONSTANT SPEED INSTEAD OF THIS METHOD?
  public double getChargeTime(Supplier<ArmState> position) {
    switch(position.get()) {
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
      topPIDController.setReference(0.0, ControlType.kVoltage);
      topShooterMotor.stopMotor();
      bottomPIDController.setReference(0.0, ControlType.kVoltage);
      bottomShooterMotor.stopMotor();
      intakeMotor.stopMotor();
    });
  }


  public static boolean noteSensorTriggered() {
    return noteSensor.get();
  }


  @Override
  public void periodic() {
    // If the wrist PID or setpoint values are different from SmartDashboard, use the new values
    if (topP != SmartDashboard.getNumber("topShooter P", 0)) {
      topP = SmartDashboard.getNumber("topShooter P", 0);
      topPIDController.setP(topP);
    }
    if (topFF != SmartDashboard.getNumber("topShooter FF", 0)) {
      topFF = SmartDashboard.getNumber("topShooter FF", 0);
      topPIDController.setI(topFF);
    }
    if (topSetPoint != SmartDashboard.getNumber("topShooter Set Point", 0)) {
      topSetPoint = SmartDashboard.getNumber("topShooter Set Point", 0);
      topPIDController.setReference(topSetPoint, ControlType.kVelocity);
    }
    if (bottomP != SmartDashboard.getNumber("bottomShooter P", 0)) {
      bottomP = SmartDashboard.getNumber("bottomShooter P", 0);
      bottomPIDController.setD(bottomP);
    }
    if (bottomFF != SmartDashboard.getNumber("bottomShooter FF", 0)) {
      bottomFF = SmartDashboard.getNumber("bottomShooter FF", 0);
      bottomPIDController.setFF(bottomFF);
    }
    if (bottomSetPoint != SmartDashboard.getNumber("bottomShooter Set Point", 0)) {
      bottomSetPoint = SmartDashboard.getNumber("bottomShooter Set Point", 0);
      bottomPIDController.setReference(bottomSetPoint, ControlType.kVelocity);
    }

    SmartDashboard.putNumber("topShooterMotor Velocity", topShooterEncoder.getVelocity());
    SmartDashboard.putNumber("bottomShooterMotor Velocity", bottomShooterEncoder.getVelocity());
  }

}