// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotorSubsystem extends SubsystemBase {

  private CANSparkMax neo550 = new CANSparkMax(28, MotorType.kBrushless);

  /** Creates a new TestMotor. */
  public TestMotorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void setMotor(double speed)
  {
    neo550.set(speed);
  }

  public Command PlaySpeakerCommand(double speed) 
  {
    return new StartEndCommand(() -> this.setMotor(speed), () -> this.setMotor(0), this);
  }
}
