// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  /* Constants for the swerve chassis */
  public static class SwerveConstants {
    public static final double MAX_SPEED  = 5.4; // maximum m/s for the robot
    public static final double PATHPLANNER_TRANS_KP = 1; // Translational P for the pathplanner PID (default 5?)
    public static final double LOOP_TIME  = 0.13; // in seconds, 20ms + 110ms spark max velocity lag
  }

  /* Constants for the controllers */
  public static class OperatorConstants {
    public static final int kDriverControllerPort   = 0;
    public static final int kOperatorControllerPort = 1;
    /* Deadbands */
    public static final double LEFT_X_DEADBAND  = 0.02;
    public static final double LEFT_Y_DEADBAND  = 0.02;
    public static final double RIGHT_X_DEADBAND = 0.02;
    public static final double RIGHT_Y_DEADBAND = 0.02;
    /* Speed Modes */
    public static final double kFastModeSpeed = 1;
    public static final double kMidModeSpeed  = 0.6;
    public static final double kSlowModeSpeed = 0.3;
  }

  /* Constants for the arm subsystem */
  public static class ArmConstants{
    // All of our PID Postions for the arm
    public static enum ArmState {IDLE, FLOOR, SOURCE, AMP, SHOOT_SUB, SHOOT_PODIUM, SHOOT_HORIZONTAL, CLIMB_1, CLIMB_2, TRAP, MANUAL};
    // Motor IDs
    public static final int kElbowMotorLeaderID   = 20;
    public static final int kElbowMotorFollowerID = 21;
    public static final int kWristMotorID         = 22;
    // NominalVoltage
    public static final double kElbowMotorNominalVoltage = 10.5;
    public static final double kWristMotorNominalVoltage = 10.5;
    // Encoder Conversion Factor
    //TODO: INCORRECT? CHECK IF IT IS
    public static final double kElbowEncoderFactor = 180;
    // Elbow and wrist PID + PID max speed
    // TODO: TUNE THESE
    public static final double kElbowP        = 0.05;
    public static final double kElbowI        = 0.000001;
    public static final double kElbowD        = 0.1;
    public static final double kElbowMinSpeed = -0.4;
    public static final double kElbowMaxSpeed = 0.4;
    public static final double kWristP        = 0.3;
    public static final double kWristI        = 0.00005;
    public static final double kWristD        = 0.0002;
    public static final double kWristMinSpeed = -0.45;
    public static final double kWristMaxSpeed = 0.45;
    // Elbow FF gravity constant
    //public static final double kElbowS = 0;
    public static final double kElbowG = 0;
    // Elbow offset. The angle should be 0 degrees when parallel
    // TODO: SET THESE AFTER ENCODER FACTOR
    public static final double kElbowAngleOffset = 0.0;
    // Setpoints for the arm subsystem
    // First value is Elbow Angle, Second is Wrist Angle. SP = SetPoint
    // TODO: ALL THESE SETPOINTS NEED TO BE UPDATED AFTER ELBOW OFFSET
    public static final double[] kIdleAngleSP             = {-0.25, 0};
    public static final double[] kFloorAngleSP            = {0, 41.5 };
    public static final double[] kSourceAngleSP           = {0,9.1};
    public static final double[] kAmpAngleSP              = {-112, 44.2};
    public static final double[] kSpeakerSubwooferAngleSP = {0, 41.5};
    public static final double[] kSpeakerPodiumAngleSP    = {-34.1, 49};
    public static final double[] kHorizontalAngleSP       = {-37.8, 40.5};
    public static final double[] kClimb1AngleSP           = {-100, 45};
    public static final double[] kClimb2AngleSP           = {-37.4, 83.7}; // NEEDS TO BE SET
    public static final double[] kTrapAngleSP             = {0, 0}; // NEEDS TO BE SET

    public static final double kManualSpeed = 0.35;
  }

  /* Constants for the box subsystem */
  public static class BoxConstants{
    // Motor IDs
    public static final int kTopShooterMotorID = 30;
    public static final int kBottomShooterMotorID = 31;
    public static final int kIntakeMotorID  = 32;
    // Sensor digital input channel
    public static final int kNoteSensorChannel = 0;
    // Nominal Voltage
    public static final double kIntakeMotorNominalVoltage  = 10.5;
    public static final double kShooterMotorNominalVoltage = 10.5;
    // Shooter motor speeds
    public static final double kTopDefaultRPM    = 2000;
    public static final double kBottomDefaultRPM = 2000;
    public static final double kTopSpeakerRPM    = 2000;
    public static final double kBottomSpeakerRPM = 2000;
    public static final double kTopAmpRPM    = 0.25;
    public static final double kBottomAmpRPM = 0.40;
    public static final double kTopHorizontalRPM    = 0.6;
    public static final double kBottomHorizontalRPM = 0.6;
    public static final double kTopYeetRPM    = 0.5;
    public static final double kBottomYeetRPM = 0.5;
    // Intake motor speeds
    public static final double kIntakeSpeed      = 1;
    public static final double kFeedSpeed        = 1;
    public static final double kRegurgitateSpeed = -0.25;
    public static final double kYeetSpeedIntake  = 1;
    // Shooter delay
    public static final double kShooterDelay     = 1.25;
    // Regurgitate time
    public static final double kRegurgitateTime  = 0.25;
    // Top shooter motor PIFF constants 
    public static final double kTopShooterP = 0.0;
    public static final double kTopShooterS = 0.0;
    public static final double kTopShooterV = kShooterMotorNominalVoltage/5676;
    //public static final double kTopShooterFF = 0.0;
    // Bottom shooter motor PIFF constants
    public static final double kBottomShooterP = 0.0;
    public static final double kBottomShooterS = 0.0;
    public static final double kBottomShooterV = kShooterMotorNominalVoltage/5676;
    //public static final double kBottomShooterFF = 0.0;
  }

  /*
  public static class LimelightConstants {
    // Wrist offsets
    public static final double wristAngleOffset = 0.0;  // in degrees
    public static final double wristPivotOffsetX = 0.00; // in Meters
    public static final double wristPivotOffsetY = 0.00; // in Meters
    // Speaker height and offset
    public static final double speakerHeight = 0.00;     // height to the middle of the speaker opening
    public static final double speakerOffsetX = 0.22;    // distance to center of speaker opening
  }
  */

}
