// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax elbowMotorLeader = new CANSparkMax(ArmConstants.kElbowMotorLeaderID,MotorType.kBrushless);
  private CANSparkMax elbowMotorFollower = new CANSparkMax(ArmConstants.kElbowMotorFollowerID,MotorType.kBrushless);
  private Encoder elbowEncoder = new Encoder(ArmConstants.kElbowEncoderIDs[0],ArmConstants.kElbowEncoderIDs[1], false);
  private SparkPIDController elbowPIDController = elbowMotorLeader.getPIDController();
  
  private CANSparkMax wristMotor = new CANSparkMax(ArmConstants.kWristMotorID,MotorType.kBrushless);
  private Encoder wristEncoder = new Encoder(ArmConstants.kWristEncoderIDs[0],ArmConstants.kWristEncoderIDs[1],false);
  private SparkPIDController wristPIDController = wristMotor.getPIDController();

  /** Creates a new Arm. */
  public ArmSubsystem() {

    elbowMotorLeader.restoreFactoryDefaults();
    elbowMotorFollower.restoreFactoryDefaults();

    wristMotor.restoreFactoryDefaults();

    elbowEncoder.reset();

    wristEncoder.reset();

    elbowEncoder.setDistancePerPulse(360/8192);
    wristEncoder.setDistancePerPulse(360/8192);

    elbowMotorFollower.follow(elbowMotorLeader);
    elbowMotorFollower.setInverted(true);

    elbowPIDController.setP(ArmConstants.kElbowP);
    elbowPIDController.setI(ArmConstants.kElbowI);
    elbowPIDController.setD(ArmConstants.kElbowD);
    elbowPIDController.setFF(ArmConstants.kElbowFF);

    wristPIDController.setP(ArmConstants.kWristP);
    wristPIDController.setI(ArmConstants.kWristI);
    wristPIDController.setD(ArmConstants.kWristD);
    wristPIDController.setFF(ArmConstants.kWristFF);

    SmartDashboard.putNumber("Elbow P", ArmConstants.kElbowP);
    SmartDashboard.putNumber("Elbow I", ArmConstants.kElbowI);
    SmartDashboard.putNumber("Elbow D", ArmConstants.kElbowD);
    SmartDashboard.putNumber("Elbow FF", ArmConstants.kElbowFF);

    SmartDashboard.putNumber("Wrist P", ArmConstants.kWristP);
    SmartDashboard.putNumber("Wrist I", ArmConstants.kWristI);
    SmartDashboard.putNumber("Wrist D", ArmConstants.kWristD);
    SmartDashboard.putNumber("Wrist FF", ArmConstants.kWristFF);
    
  }

  public Command setArmPIDCommand(double elbowAngle, double wristAngle ) {
    return runOnce(() -> {
      elbowPIDController.setReference(elbowAngle, ControlType.kPosition);
      wristPIDController.setReference(wristAngle, ControlType.kPosition);
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

    elbowPIDController.setP(SmartDashboard.getNumber("Elbow P", 0));
    elbowPIDController.setI(SmartDashboard.getNumber("Elbow I", 0));
    elbowPIDController.setD(SmartDashboard.getNumber("Elbow D", 0));
    elbowPIDController.setFF(SmartDashboard.getNumber("Elbow FF", 0));

    wristPIDController.setP(SmartDashboard.getNumber("Wrist P", 0));
    wristPIDController.setI(SmartDashboard.getNumber("Wrist I", 0));
    wristPIDController.setD(SmartDashboard.getNumber("Wrist D", 0));
    wristPIDController.setFF(SmartDashboard.getNumber("Wrist FF", 0));

    SmartDashboard.putNumber("Elbow Angular Velocity", elbowEncoder.getRate());
    SmartDashboard.putNumber("Elbow Angle", elbowEncoder.getDistance());
    SmartDashboard.putNumber("Wrist Angular Velocity", wristEncoder.getRate());
    SmartDashboard.putNumber("Wrist Angle", wristEncoder.getDistance());
  }

}
