// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Action;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants;
import frc.robot.RobotInfo;
import frc.robot.Auto.Paths;
import frc.robot.Auto.Action.CreateAndFollowMultipleTrajectory.PathType;
import frc.robot.Subsystems.Drive;

/** Add your docs here. */
public class CreatePathWeaverTrajectory implements Action {
    Trajectory trajectory;
    Trajectory printTrajectory;
    PathType _pathType;
    Drive mDrive;
    boolean notUsePW;
    TrajectoryConfig config;
    DifferentialDriveVoltageConstraint voltageConstraint;

    public CreatePathWeaverTrajectory(PathType pathType){
        _pathType = pathType;
        mDrive = Drive.getInstance();
    }

    @Override
    public void start() {
        RobotInfo.fieldSim = null;
        RobotInfo.fieldSim = new Field2d();
        voltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.kDriveS, 
                                                                                              Constants.kDriveV, 
                                                                                              Constants.kDriveA), 
                                                                   Constants.kDriveKinematics, 10);
        
        config = new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAcceleration)
                                      .setKinematics(Constants.kDriveKinematics) // Add kinematics to ensure max speed is actually obeyed
                                      .addConstraint(voltageConstraint);
        notUsePW = false;
        mDrive.stopDrive();
        mDrive.resetRamsete();
        String trajectoryJSON;
        switch (_pathType){
            case GALACTICSEARCHA:
                trajectoryJSON = new Random().nextInt(2) == 1 ? "paths/GalacticARed.wpilib.json" : "paths/GalacticABlue.wpilib.json";
                break;
            case GALACTICSEARCHB:
                trajectoryJSON = new Random().nextInt(2) == 1 ? "paths/GalacticBRed.wpilib.json" : "paths/GalacticBBlue.wpilib.json";
                break;
            case SLOLOM:
                trajectoryJSON = "paths/Slalom_v6.wpilib.json";
                notUsePW = false;
                break;
            case BOUNCE:
                trajectoryJSON = "paths/Bounce.wpilib.json";
                break;
            case BARRELRACING:
                trajectoryJSON = "paths/BarrelRacing_v1.wpilib.json";
                break;
            default:
                trajectoryJSON = "paths/Slolom.wpilib.json";
        }
        if(notUsePW){
            //trajectory = TrajectoryGenerator.generateTrajectory(Paths.manuelSlalomStart, Paths.manuelSlalomWaypoints, Paths.manuelSlalomEnd, config);
            trajectory = TrajectoryGenerator.generateTrajectory(Paths.manuelClampedSlamom, config);
            /*trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()),
                                                                List.of(new Translation2d(1, 0), new Translation2d(1, 1)),
                                                                new Pose2d(0, 0, new Rotation2d()),
                                                                config);*/
            printTrajectory = trajectory.relativeTo(Constants.glassOrigin);
        }
        else{
            trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } 
            catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
        }
        printTrajectory = trajectory;
        mDrive.constructOdometry(mDrive.getFusedGyroRotation2D());
        mDrive.createRamseteManager(trajectory);
        mDrive.resetOdometry(trajectory.getInitialPose());
        String objectApexName = "Trajectory-";
        RobotInfo.trajectory = trajectory;
        List<Pose2d> objectPoses = printTrajectory.getStates().stream()
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
        System.out.println("Lenght of waypoints is " + listSize);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {
    }
}
