// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Action;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Vision;

/** Add your docs here. */
public class CreateFieldRelativeTrajectory implements Action{

    Drive mDrive;
    Vision mVision;
    Trajectory trajectory;
    DifferentialDriveVoltageConstraint voltageConstraint;
    Trajectory trajectoryPW;
    Trajectory trajectoryManual;
    TrajectoryConfig config;

    public CreateFieldRelativeTrajectory(){
        mDrive = Drive.getInstance();
        mVision = Vision.getInstance();
    }

    @Override
    public void start() {
        System.out.println("!CREATING TRAJECTORY!");
        mDrive.stopDrive();
        mDrive.resetRamsete();
        double[] visionInfo = mVision.getInfo();
        voltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.kDriveS, 
                                                                                              Constants.kDriveV, 
                                                                                              Constants.kDriveA), 
                                                                   Constants.kDriveKinematics, 10);
        
        config = new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAcceleration)
                                      .setKinematics(Constants.kDriveKinematics) // Add kinematics to ensure max speed is actually obeyed
                                      .addConstraint(voltageConstraint); // Apply the voltage constraint
        /**String trajectoryJSON = "paths/YourPath.wpilib.json"; // change the path name
        trajectoryPW = new Trajectory();
        try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectoryPW = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } 
        catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }*/
        Translation2d startingPos = estimateStartingPosition(visionInfo[2]);
        Drive.intakeOnCheckpoint = convertToRobotRelativePos(Constants.intakeStartPoint, startingPos);
        Drive.intakeOffCheckpoit = convertToRobotRelativePos(Constants.intakeEndPoint, startingPos);
        Pose2d endPose = new Pose2d(convertToRobotRelativePos(Constants.trajectoryEndPoint, startingPos), Constants.endingRotation);
        System.out.println("Starting Pos\nX is " + startingPos.getX() + "  Y is " + startingPos.getY());
        System.out.println("1 Pos is " + Drive.intakeOnCheckpoint);
        System.out.println("2 Pos is " + Drive.intakeOffCheckpoit);
        System.out.println("end pos " + endPose);
        /*trajectoryManual = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0 , mDrive.getFusedGyroRotation2D()),
                                                                  List.of(
                                                                        firstCheckpoint,
                                                                        secondCheckpoint),
                                                                  endPose,
                                                                  config);*/
        trajectoryManual = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 0 , mDrive.getFusedGyroRotation2D()),
                                                                new Pose2d(Drive.intakeOnCheckpoint, Rotation2d.fromDegrees(-180)),
                                                                new Pose2d(Drive.intakeOffCheckpoit, Rotation2d.fromDegrees(-180)),
                                                                endPose),
                                                                config);
        mDrive.constructOdometry(mDrive.getFusedGyroRotation2D());
        mDrive.createRamseteManager(trajectoryManual);
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
        System.out.println("!CREATED TRAJECTORY!");
    }

    /**
     * Estimate starting position using vision
     * @param ty
     * @return Robots 
     */
    public Translation2d estimateStartingPosition(double ty){
        double distance = mVision.estimateDistanceFromAngle(ty);
        double gyroRadian = Math.toRadians(mDrive.getGyroAngle());
        double secondAngleRadian = Math.toRadians(90 - mDrive.getGyroAngle());
        double distance_y = distance * Math.sin(gyroRadian);
        double distance_x = distance * Math.sin(secondAngleRadian);
        System.out.println("Gyro " + gyroRadian + " Second Angle " + secondAngleRadian
        + " X is " + distance_x + " Y is" + distance_y);
        Translation2d startingPos = new Translation2d(distance_x, distance_y);
        startingPos = startingPos.plus(Constants.kVisionTargetPos);
        return startingPos;
    }

    public Translation2d convertToRobotRelativePos(Translation2d fieldRelativePose, Translation2d robotFieldRelativePos){
        Translation2d pos = fieldRelativePose.minus(robotFieldRelativePos).unaryMinus(); // maybe unary minus
        return pos;
    }
}
