// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Constants;
import frc.robot.Constants.BoxConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BoxSubsystem extends SubsystemBase {
  private CANSparkMax shooterMotor = new CANSparkMax(BoxConstants.kShooterMotorID, MotorType.kBrushless);
  private CANSparkMax intakeMotor = new CANSparkMax(BoxConstants.kIntakeMotorID, MotorType.kBrushless);


  /** Creates a new Box. */
  public BoxSubsystem() {
    intakeMotor.restoreFactoryDefaults();
    shooterMotor.restoreFactoryDefaults();

    intakeMotor.enableVoltageCompensation(BoxConstants.kIntakeMotorNominalVoltage);
    shooterMotor.enableVoltageCompensation(BoxConstants.kShooterMotorNominalVoltage);

    shooterMotor.setInverted(true);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("isForwardLimitSwitchPressed", isForwardLimitSwitchPressed()); //might be reverse
  }


  public Command intakeCommand(boolean reversed) {
    return startEnd(
      () -> intakeMotor.set(reversed ? -BoxConstants.kIntakeSpeed : BoxConstants.kIntakeSpeed),
      () -> intakeMotor.set(0)
    );
  }


  public Command prepareShootCommand() {
    return runOnce(() -> shooterMotor.set(BoxConstants.kShooterSpeed));
  }


  public Command shootCommand() {
    return startEnd(
      () -> intakeMotor.set(BoxConstants.kFeedSpeed), 
      () -> stopCommand()
    );
  }


  public Command shootCommand2() {
    //shooterMotor.set(Constants.BoxConstants.kShooterSpeed);
    //intakeMotor.set(Constants.BoxConstants.kIntakeSpeed);
    return runOnce(() -> intakeMotor.set(BoxConstants.kFeedSpeed)).withTimeout(1).andThen(() -> shooterMotor.set(BoxConstants.kFeedSpeed), this)
    .handleInterrupt(() -> stopCommand()
    );
  }


  public Command stopCommand() {
    return runOnce(
      () -> {
        intakeMotor.set(0);
        shooterMotor.set(0);
      }
    );
  }


  public boolean isForwardLimitSwitchPressed() {
    return intakeMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
  }

}