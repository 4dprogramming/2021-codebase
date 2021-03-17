// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

/** Add your docs here. */
public final class Utils {
    /**
     * Return deadband value
     * @param value
     * @param minValue
     * @param maxValue
     * @return Value
     */
    public static double applyDeadband(double value, double minValue, double maxValue){
        if(value > maxValue)
            return maxValue;
        else if(value<minValue){
            return minValue;
        }
        else{
            return value;
        }
    }
    /**
     * Tolerance
     * @param value
     * @param control_value
     * @param tolerance
     * @return value = +- control_value
     */
    public static boolean tolerance(double value,double control_value, double tolerance){
        return ((value >= (control_value - tolerance)) && (value <= (control_value + tolerance)));
    }

    public static boolean isRobotInPosition(Pose2d robotPose, Pose2d desiredPose, Translation2d tolerance){
        return (tolerance(robotPose.getX(), desiredPose.getX(), tolerance.getX()) && tolerance(robotPose.getY(), desiredPose.getY(), tolerance.getY()));
    }

    public static boolean isRobotInPosition(Pose2d robotPose, Translation2d desiredPose, Translation2d tolerance){
        return (tolerance(robotPose.getX(), desiredPose.getX(), tolerance.getX()) && tolerance(robotPose.getY(), desiredPose.getY(), tolerance.getY()));
    }

    /**
     * Map one value from a range to another
     * @param value
     * @param inputStart
     * @param inputEnd
     * @param outputStart
     * @param outputEnd
     * @return
     */
    public static double map(double value, double inputStart, double inputEnd, double outputStart, double outputEnd){
        double deltaIn = inputEnd - inputStart;
        double deltaOut = outputEnd - outputStart;
        double scale = deltaOut / deltaIn;
        double negIn = -1 * inputStart;
        double offset = (negIn * scale) + outputStart;
        double finalNumber = (value * scale) + offset;
        return finalNumber;
    }

    public static void printTrajectoryToDashboard(Trajectory trajectory){
        String objectApexName = "Trajectory-";
        List<Pose2d> objectPoses = trajectory.getStates().stream()
                                    .map(state -> state.poseMeters)
                                    .collect(Collectors.toList());
        
        for(int a = 0; a < objectPoses.size(); a++){
            if (a % 2 == 0){
                objectPoses.remove(a);
            }
        }
        int listSize = objectPoses.size();
        ArrayList<List<Pose2d>> partitions = new ArrayList<List<Pose2d>>();
        int partitionSize = 16;
        for (int i = 0; i < listSize; i += partitionSize) {
            partitions.add(objectPoses.subList(i, Math.min(i + partitionSize, objectPoses.size())));
        }
        for (int a = 0; a < partitions.size(); a++){
            System.out.println("a is" + a + "Lenght of array is" + partitions.get(a).size());
            RobotInfo.fieldSim.getObject(objectApexName + a).setPoses(partitions.get(a));
        }
    }

    /**
     * Print Trajectory But Multiple of Them @see {@link #printTrajectoryToDashboard(Trajectory)}
     * @param trajectory
     */
    public static void printTrajectoriesToDashboard(List<Trajectory> trajectories, Field2d fieldSim){
        String apexName = "Trajectory";
        int partitionSize = 16;
        for(int z = 0; z < trajectories.size(); z++){
            String objectApexName = apexName + z + "-"; 
            Trajectory trajectory = trajectories.get(z);
            List<Pose2d> objectPoses = trajectory.getStates().stream()
                                        .map(state -> state.poseMeters)
                                        .collect(Collectors.toList());
            for(int a = 0; a < objectPoses.size(); a++){
                if (a % 2 == 0){
                    objectPoses.remove(a);
                }
            }
            int listSize = objectPoses.size();
            ArrayList<List<Pose2d>> partitions = new ArrayList<List<Pose2d>>();
            for (int i = 0; i < listSize; i += partitionSize) {
                partitions.add(objectPoses.subList(i, Math.min(i + partitionSize, objectPoses.size())));
            }
            for (int a = 0; a < partitions.size(); a++){
                fieldSim.getObject(objectApexName + a).setPoses(partitions.get(a));
            }
        }
        
    }

    public static Trajectory readTrajectoryFromPW(String trajectoryLocation){
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryLocation);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } 
        catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryLocation, ex.getStackTrace());
            return null;
        }
    }
}
