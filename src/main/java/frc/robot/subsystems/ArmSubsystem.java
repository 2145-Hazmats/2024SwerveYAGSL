// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Encoder;
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

    elbowMotorFollower.follow(elbowMotorLeader);
    elbowMotorFollower.setInverted(true);


    


  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
