// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Action;

import java.util.stream.Collectors;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotInfo;
import frc.robot.Subsystems.Drive;

/** Add your docs here. */
public class FollowTrajectoryAction implements Action {
    Drive mDrive;
    boolean finished = false;

    public FollowTrajectoryAction(){
        mDrive = Drive.getInstance();
    }

    @Override
    public void done() {
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        System.out.println("!FOLLOWING TRAJECTORY!");
        mDrive.updateOdometry();
        NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
        NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
        var translation = mDrive.odometry.getPoseMeters().getTranslation();
        m_xEntry.setNumber(translation.getX());
        m_yEntry.setNumber(translation.getY());
        System.out.println("Current pos is : " + translation);
        RobotInfo.fieldSim.setRobotPose(mDrive.getPose());//.relativeTo(Constants.glassOrigin));
        SmartDashboard.putData(RobotInfo.fieldSim);
        finished = mDrive.followTrajectory();
    }
    
}
