// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;


public final class Constants {
  
  public static class SwerveConstants {
    public static final double MAX_SPEED = 1;            // m/s for the entire robot
    /* PathPlanner constants */
    public static final double MAX_MODULE_SPEED     = 1; // m/s for an individual module
    public static final double PATHPLANNER_TRANS_KP = 1; // Translational P for the pathplanner PID (default 5?)
    /* Unused constants */
    public static final double ROBOT_MASS = 45 * 0.453592; // mass in kg
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(24)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; // in seconds, 20ms + 110ms spark max velocity lag
  }
  

  public static class OperatorConstants {
    public static final int kDriverControllerPort   = 0;
    public static final int kOperatorControllerPort = 1;
    /* Deadbands */
    public static final double LEFT_X_DEADBAND  = 0.02;
    public static final double LEFT_Y_DEADBAND  = 0.02;
    public static final double RIGHT_X_DEADBAND = 0.02;
    public static final double RIGHT_Y_DEADBAND = 0.02;
  }
}
