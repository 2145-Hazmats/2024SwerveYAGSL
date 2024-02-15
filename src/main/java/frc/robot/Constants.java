// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  
  public static class SwerveConstants {
    public static final double MAX_SPEED = 3.70;         // maximum m/s for the robot
    /* PathPlanner constants */
    public static final double PATHPLANNER_TRANS_KP = 1; // Translational P for the pathplanner PID (default 5?)
    /* Unused constants */
    //public static final double ROBOT_MASS = 45 * 0.453592; // mass in kg
    //public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(24)), ROBOT_MASS);
    //public static final double LOOP_TIME  = 0.13; // in seconds, 20ms + 110ms spark max velocity lag
  }
  

  public static class OperatorConstants {
    public static final int kDriverControllerPort   = 0;
    public static final int kOperatorControllerPort = 1;
    /* Deadbands */
    public static final double LEFT_X_DEADBAND  = 0.03;
    public static final double LEFT_Y_DEADBAND  = 0.03;
    public static final double RIGHT_X_DEADBAND = 0.01;
    public static final double RIGHT_Y_DEADBAND = 0.01;

    public static final double kSlowModeSpeed = 0.5;
    public static final double kFastModeSpeed = 1;
  }

  public static class ArmConstants{
    public static final int kElbowMotorLeaderID = 30;
    public static final int kElbowMotorFollowerID = 31;
    public static final int[] kElbowEncoderIDs = {1,2};
    public static final int kWristMotorID = 32;
    public static final int[] kWristEncoderIDs = {1,2};

    public static final double kElbowP = 0;
    public static final double kElbowI = 0;
    public static final double kElbowD = 0;
    public static final double kElbowFF = 0;
    public static final double kElbowMaxAcceleration = 0;
    public static final double kElbowMaxVelocity = 0;

    public static final double kWristP = 0;
    public static final double kWristI = 0;
    public static final double kWristD = 0;
    public static final double kWristFF = 0;
    public static final double kWristMaxAcceleration = 0;
    public static final double kWristMaxVelocity = 0;

    // First value is Elbow Angle, Second is Wrist Angle. SP = SetPoint
    public static final double[] FloorAngleSP = {0,0};
    public static final double[] SourceAngleSP = {0,0};
    public static final double[] IdleAngleSP = {0,0};
    public static final double[] SpeakerSubwooferAngleSP = {0,0};
    public static final double[] SpeakerPodiumAngleSP = {0,0};
    public static final double[] TrapAngleSP = {0,0};
    public static final double[] AmpAngleSP = {0,0};
    public static final double[] HorizontalAngleSP = {0,0};
    
  }

  public static class BoxConstants{
    public static final int kShooterMotorID = 60;
    public static final int kIntakeMotorID = 61;
    public static final int kForwardLimitSwitchID = 0;
    public static final double kIntakeMotorNominalVoltage = 11.5;
    public static final double kShooterMotorNominalVoltage = 11.5;

    public static final double kIntakeSpeed = 0.5;
    public static final double kShooterSpeed = 0.5;
    public static final double kFeedSpeed = 0.5;
  }
}
