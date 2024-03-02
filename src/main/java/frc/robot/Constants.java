// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


public final class Constants {
  /* Constants for the swerve chassis */
  public static class SwerveConstants {
    public static final double MAX_SPEED  = 4.46; // maximum m/s for the robot
    public static final double PATHPLANNER_TRANS_KP = 1; // Translational P for the pathplanner PID (default 5?)
    public static final double LOOP_TIME  = 0.13; // in seconds, 20ms + 110ms spark max velocity lag
  }

  /* Constants for the controllers */
  public static class OperatorConstants {
    public static final int kDriverControllerPort   = 0;
    public static final int kOperatorControllerPort = 1;
    /* Deadbands */
    public static final double LEFT_X_DEADBAND  = 0.02; //0.04
    public static final double LEFT_Y_DEADBAND  = 0.02; //0.04
    public static final double RIGHT_X_DEADBAND = 0.02;
    public static final double RIGHT_Y_DEADBAND = 0.02;
    public static final double kVisionModeDeadband = 0.10;
    /* Speed Modes */
    public static final double kFastModeSpeed = 1;
    public static final double kMidModeSpeed  = 0.5;
    public static final double kSlowModeSpeed = 0.25;
  }

  /* Constants for the arm subsystem */
  public static class ArmConstants{
    // All of our PID Postions for the arm
    public static enum ArmState {IDLE, FLOOR, SOURCE, SHOOT_SUB, SHOOT_PODIUM, SHOOT_HORIZONTAL, CLIMBING_POSITION, AMP, TRAP, MANUAL}; 
    // Motor IDs
    public static final int kElbowMotorLeaderID   = 20;
    public static final int kElbowMotorFollowerID = 21;
    public static final int kWristMotorID         = 22;
    // NominalVoltage
    public static final double kElbowMotorNominalVoltage = 12;
    public static final double kWristMotorNominalVoltage = 12;
    // Elbow PID + PID max speed
    public static final double kElbowP        = 0.05; //0.1
    public static final double kElbowI        = 0.000001; //0.0001
    public static final double kElbowD        = 0.1; //1
    public static final double kElbowFF       = 0;
    public static final double kElbowMinSpeed = -0.4; //-0.4
    public static final double kElbowMaxSpeed = 0.4;
    public static final double kWristP        = 0.3;
    public static final double kWristI        = 0.00005;
    public static final double kWristD        = 0.0002;
    public static final double kWristFF       = 0;
    public static final double kWristMinSpeed = -0.45; //-0.55
    public static final double kWristMaxSpeed = 0.45; //0.55
    // Setpoints for the arm subsystem
    // First value is Elbow Angle, Second is Wrist Angle. SP = SetPoint
    public static final double[] kFloorAngleSP            = {0, 41.5 }; // 0 , 37
    public static final double[] kSourceAngleSP           = {0,9.1};
    public static final double[] kIdleAngleSP             = {-0.25, 0};
    public static final double[] kSpeakerSubwooferAngleSP = {0, 41.5};
    public static final double[] kSpeakerPodiumAngleSP    = {-34.1, 49};
    public static final double[] kTrapAngleSP             = {0, 0};
    public static final double[] kAmpAngleSP              = {-112, 44.8};// -110,42
    public static final double[] kHorizontalAngleSP       = {-37.8, 40.5};
    public static final double[] kClimbingAngleSP         = {-95.2, 32};
  }

  /* Constants for the box subsystem */
  public static class BoxConstants{
    // Motor IDs
    public static final int kShooterMotorID = 30;
    public static final int kTheOtherShooterMotorID = 31;
    public static final int kIntakeMotorID  = 32;
    // Nominal Voltage
    public static final double kIntakeMotorNominalVoltage  = 12;
    public static final double kShooterMotorNominalVoltage = 12;
    // Shooter motor speeds
    public static final double kSpeakerShootSpeed = 0.36;
    public static final double kAmpShootSpeed = 0.15;
    public static final double kHorizontalShootSpeed = 0.6;
    public static final double kDefaultShootSpeed = 0.36;
    // Intake motor speeds
    public static final double kIntakeSpeed      = 1;
    public static final double kFeedSpeed        = 1;
    public static final double kRegurgitateSpeed = -0.75;
    // Shooter delay
    public static final double kShooterDelay = 1.25;
    public static final double kShooteDelayAmp = .1;
  }

}
