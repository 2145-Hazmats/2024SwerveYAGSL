// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;


public class LimelightSubsystem extends SubsystemBase {
    // We might not need this, but I included it for later if we have
    // commands that need the SwerveSubsystem in the LimelightSubsystem
    private final SwerveSubsystem m_swerve;

    /** Creates a new Limelight. */
    public LimelightSubsystem(SwerveSubsystem m_subsystem) {
        m_swerve = m_subsystem;
    }


    /**
     * Gets the yaw angle from the Limelight to the middle of the apriltag
     */
    public double getTargetRotation() {
        // Returns a rotation value if the limelight has a valid apriltag target
        if (LimelightHelpers.getTV("") == true) {
            return getCameraTransform(4); // I hope this is yaw
        }
        // Otherwise, don't rotate the robot
        return 0;
    }


    // Custom method to get camtran from network table
    // 0,                   1,         2,                  3,     4,   5
    // x(lateral distance), y(height), z(length distance), pitch, yaw, roll
    public double getCameraTransform(int index) {
        double[] camtrans = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
        return camtrans[index];
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("getLatency_Pipeline", LimelightHelpers.getLatency_Pipeline(""));

        SmartDashboard.putBoolean("get TV", LimelightHelpers.getTV(""));
        SmartDashboard.putNumber("getFiducialID", LimelightHelpers.getFiducialID(""));
        SmartDashboard.putNumber("get Target Area", LimelightHelpers.getTA(""));

        SmartDashboard.putNumber("getBotPose2d X", LimelightHelpers.getBotPose2d("").getX());
        SmartDashboard.putNumber("getBotPose2d Y", LimelightHelpers.getBotPose2d("").getY());

        SmartDashboard.putNumber("Target Distance", -(getCameraTransform(2)));
        SmartDashboard.putNumber("Target Lateral", getCameraTransform(0));
        SmartDashboard.putNumber("Target Height", getCameraTransform(1));
        SmartDashboard.putNumber("Target Rotation", getCameraTransform(4));
    }

}