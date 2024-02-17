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

    public static final double kFastModeSpeed = 1;
    public static final double kMidModeSpeed = 0.5;
    public static final double kSlowModeSpeed = 0.1;
  }

  public static class ArmConstants{
    public static final int kElbowMotorLeaderID = 20;
    public static final int kElbowMotorFollowerID = 21;
    public static final int kWristMotorID = 22;
    public static final double kElbowMotorNominalVoltage = 12;
    public static final double kWristMotorNominalVoltage = 12;

    public static final double kElbowP = 0.05; //0.1
    public static final double kElbowI = 0.000001; //0.0001
    public static final double kElbowD = 0.1; //1
    public static final double kElbowFF = 0;
    public static final double kElbowMinSpeed = -1.0;
    public static final double kElbowMaxSpeed = 1.0;

    public static final double kWristP = 0.1;
    public static final double kWristI = 0.0001;
    public static final double kWristD = 1;
    public static final double kWristFF = 0;
    public static final double kWristMinSpeed = -0.1;
    public static final double kWristMaxSpeed = 0.1;

    // First value is Elbow Angle, Second is Wrist Angle. SP = SetPoint
    public static final double[] kFloorAngleSP = {0, 0};
    public static final double[] kSourceAngleSP = {15, 30};
    public static final double[] kIdleAngleSP = {30, 15};
    public static final double[] kSpeakerSubwooferAngleSP = {45, 60};
    public static final double[] kSpeakerPodiumAngleSP = {-20, 30};
    public static final double[] kTrapAngleSP = {25, 25};
    public static final double[] kAmpAngleSP = {-15, 20};
    public static final double[] kHorizontalAngleSP = {30, 30};

    public static final double[][] kMasterAngleArray = {
        kFloorAngleSP, kSourceAngleSP, kIdleAngleSP, kSpeakerSubwooferAngleSP,
        kSpeakerPodiumAngleSP, kTrapAngleSP, kAmpAngleSP, kHorizontalAngleSP
    };
  }

  public static class BoxConstants{
    public static final int kShooterMotorID = 30;
    public static final int kIntakeMotorID = 31;
    public static final int kForwardLimitSwitchID = 0;
    public static final double kIntakeMotorNominalVoltage = 11.5;
    public static final double kShooterMotorNominalVoltage = 11.5;

    public static final double kIntakeSpeed = 1;
    public static final double kShooterSpeed = 0.1;
    public static final double kFeedSpeed = 0.5;
  }

}
