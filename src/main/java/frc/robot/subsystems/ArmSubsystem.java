// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class ArmSubsystem extends SubsystemBase {
  // Declare and intialize motors and encoders
  private final CANSparkMax elbowMotorLeader = new CANSparkMax(ArmConstants.kElbowMotorLeaderID, MotorType.kBrushless);
  private final CANSparkMax elbowMotorFollower = new CANSparkMax(ArmConstants.kElbowMotorFollowerID, MotorType.kBrushless);
  private RelativeEncoder elbowEncoder = elbowMotorLeader.getAlternateEncoder(8192);

  private final CANSparkMax wristMotor = new CANSparkMax(ArmConstants.kWristMotorID, MotorType.kBrushless);
  //private RelativeEncoder wristEncoder = wristMotor.getAlternateEncoder(8192);
  private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
  // Get the PIDController object for the elbow and wrist
  private SparkPIDController elbowPIDController = elbowMotorLeader.getPIDController();
  private SparkPIDController wristPIDController = wristMotor.getPIDController();
  // Variables used during SmartDashboard changes
  private double elbowP, elbowI, elbowD, elbowFF, elbowSetPoint = 0;
  private double wristP, wristI, wristD, wristFF, wristSetPoint = 0;


  /** Creates a new Arm. */
  public ArmSubsystem() {
    // Restore factory defaults
    elbowMotorLeader.restoreFactoryDefaults();
    elbowMotorFollower.restoreFactoryDefaults();
    wristMotor.restoreFactoryDefaults();
    // Have the second elbow motor follow the first
    elbowMotorFollower.follow(elbowMotorLeader, true);
    // Enable voltage compensation
    elbowMotorLeader.enableVoltageCompensation(ArmConstants.kElbowMotorNominalVoltage);
    elbowMotorFollower.enableVoltageCompensation(ArmConstants.kElbowMotorNominalVoltage);
    wristMotor.enableVoltageCompensation(ArmConstants.kWristMotorNominalVoltage);

    // Setup encoders
    elbowEncoder.setPositionConversionFactor(180);
    wristEncoder.setPositionConversionFactor(1);
    elbowEncoder.setPosition(0);
    wristEncoder.setPosition(0);
    // Invert the elbow encoder. Mandatory
    elbowEncoder.setInverted(true);

    // Change PIDController FeedbackDevice from the integrated encoder to the alternate encoder
    elbowPIDController.setFeedbackDevice(elbowEncoder);
    wristPIDController.setFeedbackDevice(wristEncoder);

    // Setup the elbow PIDController
    elbowPIDController.setP(ArmConstants.kElbowP);
    elbowPIDController.setI(ArmConstants.kElbowI);
    elbowPIDController.setD(ArmConstants.kElbowD);
    elbowPIDController.setFF(ArmConstants.kElbowFF);
    elbowPIDController.setOutputRange(ArmConstants.kElbowMinSpeed, ArmConstants.kElbowMaxSpeed);

    // Setup the wrist PIDController
    wristPIDController.setP(ArmConstants.kWristP);
    wristPIDController.setI(ArmConstants.kWristI);
    wristPIDController.setD(ArmConstants.kWristD);
    wristPIDController.setFF(ArmConstants.kWristFF);
    wristPIDController.setOutputRange(ArmConstants.kWristMinSpeed, ArmConstants.kWristMaxSpeed);

    // Put Elbow PIDs on SmartDashboard
    SmartDashboard.putNumber("Elbow P", ArmConstants.kElbowP);
    SmartDashboard.putNumber("Elbow I", ArmConstants.kElbowI);
    SmartDashboard.putNumber("Elbow D", ArmConstants.kElbowD);
    SmartDashboard.putNumber("Elbow FF", ArmConstants.kElbowFF);
    SmartDashboard.putNumber("Elbow Set Point", 0);
    
    // Put Wrist PIDs on SmartDashboard
    SmartDashboard.putNumber("Wrist P", ArmConstants.kWristP);
    SmartDashboard.putNumber("Wrist I", ArmConstants.kWristI);
    SmartDashboard.putNumber("Wrist D", ArmConstants.kWristD);
    SmartDashboard.putNumber("Wrist FF", ArmConstants.kWristFF);
    SmartDashboard.putNumber("Wrist Set Point", 0);
  }

  /**
   * Sets the reference angle of the elbow and wrist.
   * Until the PIDController is given another angle or ControlType, the PID will stay on.
   *
   * @param elbowAngle  The angle the elbow will rotate to and stay at.
   * @param wristAngle  The angle the wrist will rotate to and stay at.
   */
  public Command setArmPIDCommand(double elbowAngle, double wristAngle ) {
    return startEnd(
      // When the command is called, the elbow and wrist PIDController is set and updated on SmartDashboard
      () -> {
        elbowPIDController.setReference(elbowAngle, ControlType.kPosition);
        wristPIDController.setReference(wristAngle, ControlType.kPosition);
        SmartDashboard.putNumber("Elbow Set Point", elbowAngle);
        SmartDashboard.putNumber("Wrist Set Point", wristAngle);
      },
      // When the command is interrupted, the elbow and wrist go to their idle position
      () -> {
        elbowPIDController.setReference(ArmConstants.kIdleAngleSP[0], ControlType.kPosition);
        wristPIDController.setReference(ArmConstants.kIdleAngleSP[1], ControlType.kPosition);
      }
    );
  }

  /**
   * Stops the arm by setting the PIDControllers to a value of 0 with a type of ControlType.kVelocity.
   */
  public Command stopArmCommand() {
    return runOnce(() -> {
      elbowPIDController.setReference(0, ControlType.kVelocity);
      wristPIDController.setReference(0, ControlType.kVelocity);
       wristPIDController.setReference(0, ControlType.kVelocity);
    });
  }
 
  public void resetWrist() {
    wristEncoder.setPosition(0);
  };

  public Command resetWristCommand() {
    return runOnce(() -> resetWrist());
  }
 
  /**
   * Sets the elbow motor speed.
   * 
   * @param speed  The speed of the elbow motor.
   */
  public void setElbowSpeed(double speed) {
    elbowMotorLeader.set(speed);
  }

  /**
   * Sets the elbow motor speed and wrist motor speed in manual mode by giving their PIDControllers
   * a speed in ControlType.kDutyCycle mode.
   * 
   * @param wristSpeed  The speed of the wrist motor from a joystick axis.
   * @param elbowSpeed  The speed of the elbow motor from a joystick axis.
   */
  public Command manualArmCommand(DoubleSupplier wristSpeed, DoubleSupplier elbowSpeed){
    return run(()->{
      wristPIDController.setReference(wristSpeed.getAsDouble(), ControlType.kDutyCycle);
      elbowPIDController.setReference(elbowSpeed.getAsDouble(), ControlType.kDutyCycle);
    });
  }

  public Command PIDFallin(){
    return run(()->{
      wristPIDController.setReference(0, ControlType.kDutyCycle);
      elbowPIDController.setReference(0, ControlType.kDutyCycle);
    });
  }

  /**
   * Sets the wrist motor speed.
   * 
   * @param speed  The speed of the wrist motor.
   */
  public void setWristSpeed(double speed) {
    wristMotor.set(speed);
  }


  @Override
  public void periodic() {
    // If the elbow PID or setpoint values are different from SmartDashboard, use the new values
    if (elbowP != SmartDashboard.getNumber("Elbow P", 0)) {
      elbowP = SmartDashboard.getNumber("Elbow P", 0);
      elbowPIDController.setP(elbowP);
    }
    if (elbowI != SmartDashboard.getNumber("Elbow I", 0)) {
      elbowI = SmartDashboard.getNumber("Elbow I", 0);
      elbowPIDController.setI(elbowI);
    }
    if (elbowD != SmartDashboard.getNumber("Elbow D", 0)) {
      elbowD = SmartDashboard.getNumber("Elbow D", 0);
      elbowPIDController.setD(elbowD);
    }
    if (elbowFF != SmartDashboard.getNumber("Elbow FF", 0)) {
      elbowFF = SmartDashboard.getNumber("Elbow FF", 0);
      elbowPIDController.setFF(elbowFF);
    }
    if (elbowSetPoint != SmartDashboard.getNumber("Elbow Set Point", 0)) {
      elbowSetPoint = SmartDashboard.getNumber("Elbow Set Point", 0);
      elbowPIDController.setReference(elbowSetPoint, ControlType.kPosition);
    }

    // If the wrist PID or setpoint values are different from SmartDashboard, use the new values
    if (wristP != SmartDashboard.getNumber("Wrist P", 0)) {
      wristP = SmartDashboard.getNumber("Wrist P", 0);
      wristPIDController.setP(wristP);
    }
    if (wristI != SmartDashboard.getNumber("Wrist I", 0)) {
      wristI = SmartDashboard.getNumber("Wrist I", 0);
      wristPIDController.setI(wristI);
    }
    if (wristD != SmartDashboard.getNumber("Wrist D", 0)) {
      wristD = SmartDashboard.getNumber("Wrist D", 0);
      wristPIDController.setD(wristD);
    }
    if (wristFF != SmartDashboard.getNumber("Wrist FF", 0)) {
      wristFF = SmartDashboard.getNumber("Wrist FF", 0);
      wristPIDController.setFF(wristFF);
    }
    if (wristSetPoint != SmartDashboard.getNumber("Wrist Set Point", 0)) {
      wristSetPoint = SmartDashboard.getNumber("Wrist Set Point", 0);
      wristPIDController.setReference(wristSetPoint, ControlType.kPosition);
    }

    // Update SmartDashboard with elbow and wrist information
    SmartDashboard.putNumber("Elbow Angular Velocity", elbowEncoder.getVelocity());
    SmartDashboard.putNumber("Elbow Angle", elbowEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Angular Velocity", wristEncoder.getVelocity());
    SmartDashboard.putNumber("Wrist Angle", wristEncoder.getPosition());
  }

}
