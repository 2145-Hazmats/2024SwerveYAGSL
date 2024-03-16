// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmState;
import frc.robot.Constants.BoxConstants;


public class BoxSubsystem extends SubsystemBase {
  // Declare and intialize motor variables to a new instance of CANSparkMax
  private final CANSparkMax topShooterMotor = new CANSparkMax(BoxConstants.kTopShooterMotorID, MotorType.kBrushless);
  private final RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
  private final CANSparkMax bottomShooterMotor = new CANSparkMax(BoxConstants.kBottomShooterMotorID, MotorType.kBrushless);
  private final RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();
  private final CANSparkMax intakeMotor = new CANSparkMax(BoxConstants.kIntakeMotorID, MotorType.kBrushless);
  // PID controllers
  private SparkPIDController topShooterPIDController = topShooterMotor.getPIDController();
  private SparkPIDController bottomShooterPIDController = bottomShooterMotor.getPIDController();
  private SimpleMotorFeedforward topShooterFF = new SimpleMotorFeedforward(
      BoxConstants.kTopShooterS,
      BoxConstants.kTopShooterV
  );
  private SimpleMotorFeedforward bottomShooterFF = new SimpleMotorFeedforward(
      BoxConstants.kBottomShooterS,
      BoxConstants.kBottomShooterV
  );
  // Variables used during SmartDashboard changes
  private double topShooterP    = 0;
  private double bottomShooterP = 0;
  // shooterMotor variable
  private double shooterSpeed = 0; 
  // Sensor 
  private static DigitalInput noteSensor = new DigitalInput(BoxConstants.kNoteSensorChannel);
  public boolean laser;
  /** Creates a new Box. */
  public BoxSubsystem() {
    /* Motor Configuration */

    // Restore factory defaults of the Spark Max.
    // It's important to have all Spark Maxs behave the expected way, especially if we switch to a different Spark Max in the middle of a competition.
    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    // Set motor current limit
    topShooterMotor.setSmartCurrentLimit(40);
    bottomShooterMotor.setSmartCurrentLimit(40);
    intakeMotor.setSmartCurrentLimit(20);

    // Enable voltage compensation
    intakeMotor.enableVoltageCompensation(BoxConstants.kIntakeMotorNominalVoltage);
    topShooterMotor.enableVoltageCompensation(BoxConstants.kShooterMotorNominalVoltage);
    bottomShooterMotor.enableVoltageCompensation(BoxConstants.kShooterMotorNominalVoltage);

    // Reduce the frequency of the motor position sent to the roboRIO
    intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    topShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    bottomShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);

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

     /* PIDControllers */

    // Set PIDController FeedbackDevice
    topShooterPIDController.setFeedbackDevice(topShooterEncoder);
    bottomShooterPIDController.setFeedbackDevice(bottomShooterEncoder);

    // Setup the Top shooter PIDController
    topShooterPIDController.setP(BoxConstants.kTopShooterP);
    //topShooterPIDController.setFF(BoxConstants.kTopShooterFF);

    // Setup the Bottom Shooter PIDController
    bottomShooterPIDController.setP(BoxConstants.kBottomShooterP);
    //bottomShooterPIDController.setFF(BoxConstants.kBottomShooterFF);

    // Put Top Shooter PIDs on SmartDashboard
    SmartDashboard.putNumber("TopShooter P", BoxConstants.kTopShooterP);
    SmartDashboard.putNumber("TopShooter Set Point", 0); 
    
    // Put Bottom Shooter PIDs on SmartDashboard
    SmartDashboard.putNumber("BottomShooter P", BoxConstants.kBottomShooterP);
    SmartDashboard.putNumber("BottomShooter Set Point", 0);
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


  public void Yeet(double shooterSpeed, double intakeSpeed) {
  //  intakeMotor.set(Constants.BoxConstants.kYeetSpeedIntake);
  //  topShooterMotor.set(Constants.BoxConstants.kTopYeetRPM);
  //  bottomShooterMotor.set(Constants.BoxConstants.kBottomYeetRPM);
    intakeMotor.set(intakeSpeed);
    topShooterMotor.set(shooterSpeed);
    bottomShooterMotor.set(shooterSpeed);
  }

  
  public Command YeetCommand(double shooterSpeedCommand, double intakeSpeedCommand) {
    return Commands.startEnd(() -> Yeet(shooterSpeedCommand, intakeSpeedCommand), () -> Yeet(0, 0), this);
  }


  public Command ShootNoteSubwoofer() {
    return setIntakeMotorCommandThenStop(Constants.BoxConstants.kRegurgitateSpeed)
    .withTimeout(.25) 
    .andThen(setShooterMotorCommand(Constants.BoxConstants.kTopSpeakerRPM))
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
          shooterSpeed = BoxConstants.kTopSpeakerRPM;
          break;
        case AMP:
          shooterSpeed = BoxConstants.kTopAmpRPM;
          break;
        case IDLE:
          shooterSpeed = 0.0;
          break;
        case SHOOT_HORIZONTAL:
          shooterSpeed = BoxConstants.kTopHorizontalRPM;
          break;
        default:
          shooterSpeed = BoxConstants.kTopDefaultRPM;
          break;
      }

      topShooterPIDController.setReference(
          shooterSpeed,
          ControlType.kVelocity,
          0,
          topShooterFF.calculate(shooterSpeed)
      );
      bottomShooterPIDController.setReference(
          shooterSpeed,
          ControlType.kVelocity,
          0,
          bottomShooterFF.calculate(shooterSpeed)
      );

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
      shooterSpeed = 0;
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
    if (topShooterP != SmartDashboard.getNumber("TopShooter P", 0)) {
      topShooterP = SmartDashboard.getNumber("TopShooter P", 0);
      topShooterPIDController.setP(topShooterP);
    }
    if (bottomShooterP != SmartDashboard.getNumber("BottomShooter P", 0)) {
      bottomShooterP = SmartDashboard.getNumber("BottomShooter P", 0);
      bottomShooterPIDController.setP(bottomShooterP);
    }

    //This updates when the laser is broken
    laser = noteSensor.get();

    SmartDashboard.putNumber("topShooterMotor Velocity", topShooterEncoder.getVelocity());
    SmartDashboard.putNumber("bottomShooterMotor Velocity", bottomShooterEncoder.getVelocity());
    // motor.AppliedOutput() * motor.BusVoltage() gives us our real volts for sparkmax.
    SmartDashboard.putNumber("TopShooterMotorVoltage", topShooterMotor.getAppliedOutput() * topShooterMotor.getBusVoltage());
    SmartDashboard.putNumber("BottomShooterMotorVoltage", bottomShooterMotor.getAppliedOutput() * bottomShooterMotor.getBusVoltage());
    // Volts applied from FF
    SmartDashboard.putNumber("TopShooter kS Volts", BoxConstants.kTopShooterS * Math.signum(shooterSpeed));
    SmartDashboard.putNumber("TopShooter kV Volts", BoxConstants.kTopShooterV * shooterSpeed);
    SmartDashboard.putNumber("BottomShooter kS Volts", BoxConstants.kBottomShooterS * Math.signum(shooterSpeed));
    SmartDashboard.putNumber("BottomShooter kV Volts", BoxConstants.kBottomShooterV * shooterSpeed);

    SmartDashboard.putData("IR Sensor", noteSensor);
    SmartDashboard.putBoolean("IR Sensor Value", !laser);
  }

}