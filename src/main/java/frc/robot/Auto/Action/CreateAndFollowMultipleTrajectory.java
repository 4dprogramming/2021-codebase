// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Action;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Utils;
import frc.robot.Subsystems.Drive;

/** Add your docs here. */
public class CreateAndFollowMultipleTrajectory implements Action {

    public enum PathType{
        GALACTICSEARCHA,
        GALACTICSEARCHB,
        SLOLOM,
        BOUNCE,
        BARRELRACING
    }

    PathType _pathType;
    Drive mDrive;
    boolean finished;
    List<Trajectory> trajectories;
    List<String> trajectoryPaths;
    int currentTrajectory;
    Field2d fieldSim;

    public CreateAndFollowMultipleTrajectory(PathType pathType){
        _pathType = pathType;
        mDrive = Drive.getInstance();
        fieldSim = new Field2d();
        trajectories = new ArrayList<Trajectory>();
        trajectoryPaths = new ArrayList<String>();
    }

    @Override
    public void start() {
        switch (_pathType){
            case GALACTICSEARCHA:
                trajectoryPaths.add("paths/GalacticARed.wpilib.json");
                break;
            case GALACTICSEARCHB:
                trajectoryPaths.add("paths/GalacticBRed.wpilib.json");
                break;
            case SLOLOM:
                trajectoryPaths.add("paths/Slalom_v6.wpilib.json");
                break;
            case BOUNCE:
                trajectoryPaths.addAll(List.of("paths/BounceA3.wpilib.json",
                                                "paths/BounceA6.wpilib.json",
                                                "paths/BounceA9.wpilib.json",
                                                "paths/BounceExit.wpilib.json"));
                break;
            case BARRELRACING:
                trajectoryPaths.add("paths/BarrelRacing_v1.wpilib.json");
                break;
            default:
                trajectoryPaths.add("paths/Slolom.wpilib.json");
        }
        mDrive.stopDrive();
        mDrive.resetRamsete();
        for(String trajectoryPath:trajectoryPaths){
            trajectories.add(Utils.readTrajectoryFromPW(trajectoryPath));
        }
        Utils.printTrajectoriesToDashboard(trajectories, fieldSim);
        mDrive.constructOdometry(mDrive.getFusedGyroRotation2D());
        mDrive.createRamseteManager(trajectories.get(0));
        mDrive.resetOdometry(trajectories.get(0).getInitialPose());
        finished = false;
        currentTrajectory = 0;
    }

    @Override
    public void update() {
        mDrive.updateOdometry();
        fieldSim.setRobotPose(mDrive.getPose());
        SmartDashboard.putData(fieldSim);
        finished = mDrive.followTrajectory();
        if (finished){
            currentTrajectory++;
            if(currentTrajectory < trajectories.size()){
                finished = false;
                mDrive.createRamseteManager(trajectories.get(currentTrajectory));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
        mDrive.stopDrive();
    }

}
