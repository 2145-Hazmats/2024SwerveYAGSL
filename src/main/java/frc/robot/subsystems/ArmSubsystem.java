// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax elbowMotorLeader = new CANSparkMax(ArmConstants.kElbowMotorLeaderID, MotorType.kBrushless);
  private final CANSparkMax elbowMotorFollower = new CANSparkMax(ArmConstants.kElbowMotorFollowerID, MotorType.kBrushless);
  private RelativeEncoder elbowEncoder = elbowMotorLeader.getAlternateEncoder(8192);
  private SparkPIDController elbowPIDController = elbowMotorLeader.getPIDController();
  
  private final CANSparkMax wristMotor = new CANSparkMax(ArmConstants.kWristMotorID, MotorType.kBrushless);
  private RelativeEncoder wristEncoder = wristMotor.getAlternateEncoder(8192);
  private SparkPIDController wristPIDController = wristMotor.getPIDController();

  private double elbowP, elbowI, elbowD, elbowFF, elbowSetPoint = 0;
  private double wristP, wristI, wristD, wristFF, wristSetPoint = 0;

  /** Creates a new Arm. */
  public ArmSubsystem() {
    elbowMotorLeader.restoreFactoryDefaults();
    elbowMotorFollower.restoreFactoryDefaults();

    elbowMotorFollower.follow(elbowMotorLeader, true);

    wristMotor.restoreFactoryDefaults();

    elbowMotorLeader.enableVoltageCompensation(ArmConstants.kElbowMotorNominalVoltage);
    elbowMotorFollower.enableVoltageCompensation(ArmConstants.kElbowMotorNominalVoltage);
    wristMotor.enableVoltageCompensation(ArmConstants.kWristMotorNominalVoltage);


    elbowEncoder.setPositionConversionFactor(180);
    wristEncoder.setPositionConversionFactor(180);
    elbowEncoder.setPosition(0);
    wristEncoder.setPosition(0);
    elbowEncoder.setInverted(true);

    elbowPIDController.setFeedbackDevice(elbowEncoder);
    wristPIDController.setFeedbackDevice(wristEncoder);

    elbowPIDController.setP(ArmConstants.kElbowP);
    elbowPIDController.setI(ArmConstants.kElbowI);
    elbowPIDController.setD(ArmConstants.kElbowD);
    elbowPIDController.setFF(ArmConstants.kElbowFF);
    elbowPIDController.setOutputRange(ArmConstants.kElbowMinSpeed, ArmConstants.kElbowMaxSpeed);

    wristPIDController.setP(ArmConstants.kWristP);
    wristPIDController.setI(ArmConstants.kWristI);
    wristPIDController.setD(ArmConstants.kWristD);
    wristPIDController.setFF(ArmConstants.kWristFF);
    wristPIDController.setOutputRange(ArmConstants.kWristMinSpeed, ArmConstants.kWristMaxSpeed);

    SmartDashboard.putNumber("Elbow P", ArmConstants.kElbowP);
    SmartDashboard.putNumber("Elbow I", ArmConstants.kElbowI);
    SmartDashboard.putNumber("Elbow D", ArmConstants.kElbowD);
    SmartDashboard.putNumber("Elbow FF", ArmConstants.kElbowFF);
    SmartDashboard.putNumber("Elbow Set Point", 0);

    SmartDashboard.putNumber("Wrist P", ArmConstants.kWristP);
    SmartDashboard.putNumber("Wrist I", ArmConstants.kWristI);
    SmartDashboard.putNumber("Wrist D", ArmConstants.kWristD);
    SmartDashboard.putNumber("Wrist FF", ArmConstants.kWristFF);
    SmartDashboard.putNumber("Wrist Set Point", 0);
  }
  

  public Command setArmPIDCommand(double elbowAngle, double wristAngle ) {
    // This is where we should put a start end command. Start is what we have right now, and End is the default idle position
    return runOnce(() -> {
      elbowPIDController.setReference(elbowAngle, ControlType.kPosition);
      wristPIDController.setReference(wristAngle, ControlType.kPosition);
      SmartDashboard.putNumber("Elbow Set Point", elbowAngle);
      SmartDashboard.putNumber("Wrist Set Point", wristAngle);
    });
  }


  public Command stopArmCommand() {
    return runOnce(() -> {
      elbowPIDController.setReference(0, ControlType.kVelocity);
      wristPIDController.setReference(0, ControlType.kVelocity);
    });
  }


  public void setElbowSpeed(double speed) {
    elbowMotorLeader.set(speed);
  }


  public void setWristSpeed(double speed) {
    wristMotor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

    SmartDashboard.putNumber("Elbow Angular Velocity", elbowEncoder.getVelocity());
    SmartDashboard.putNumber("Elbow Angle", elbowEncoder.getPosition());
    SmartDashboard.putNumber("Wrist Angular Velocity", wristEncoder.getVelocity());
    SmartDashboard.putNumber("Wrist Angle", wristEncoder.getPosition());
  }

}
