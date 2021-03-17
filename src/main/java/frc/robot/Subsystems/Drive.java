// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

/**
 * Add your docs here.
*/ 
public class Drive {
    private static Drive mInstance = new Drive();

    public static Drive getInstance(){
        return mInstance;
    }

    public PigeonIMU pigeon;
    public Encoder leftEncoder;
    public Encoder rightEncoder;
    // Master VictorSPX
    public WPI_VictorSPX driveLMaster;
    public WPI_VictorSPX driveRMaster;

    // Follower VictorSPX
    public WPI_VictorSPX driveLTwo;
    public WPI_VictorSPX driveLThree;
    public WPI_VictorSPX driveRTwo;
    public WPI_VictorSPX driveRThree;
    public SpeedControllerGroup leftMotor;
    public SpeedControllerGroup rightMotor;

    public PIDController pid;
    public SimpleMotorFeedforward driveFeedforward;
    public DifferentialDrive differentialDrive;

    public DifferentialDriveOdometry odometry;
    public RamseteManager ramseteManager;

    public PIDController leftController = new PIDController(0, 0, 0);
    public PIDController rightController = new PIDController(0, 0, 0);

    public static Translation2d intakeOnCheckpoint;
    public static Translation2d intakeOffCheckpoit;

    private Drive(){
        // EVERYTHING IS IN METERS
        driveLMaster = new WPI_VictorSPX(Constants.driveOneLeftMotorPort);
        driveRMaster = new WPI_VictorSPX(Constants.driveOneRightMotorPort);

        driveLTwo = new WPI_VictorSPX(Constants.driveTwoLeftMotorPort);
        driveLThree = new WPI_VictorSPX(Constants.driveThreeLeftMotorPort);
        driveRTwo = new WPI_VictorSPX(Constants.driveTwoRightMotorPort);
        driveRThree = new WPI_VictorSPX(Constants.driveThreeRightMotorPort);

        leftMotor = new SpeedControllerGroup(driveLMaster, driveLTwo, driveLThree);
        rightMotor = new SpeedControllerGroup(driveRMaster, driveRTwo, driveRThree);

        differentialDrive = new DifferentialDrive(leftMotor, rightMotor);

        pigeon = new PigeonIMU(Constants.pigeonPort);
        pid = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD);
        driveFeedforward = new SimpleMotorFeedforward(Constants.kDriveS, Constants.kDriveV, Constants.kDriveA);
        leftEncoder = new Encoder(Constants.driveLeftEncPort1, Constants.driveLeftEncPort2, false, EncodingType.k1X);
        leftEncoder.setSamplesToAverage(10);
        leftEncoder.setDistancePerPulse((1/Constants.encoderCPR)*Constants.wheelPerimeter);
        rightEncoder = new Encoder(Constants.driveRightEncPort1, Constants.driveRightEncPort2, true, EncodingType.k1X);
        rightEncoder.setSamplesToAverage(10);
        rightEncoder.setDistancePerPulse((1/Constants.encoderCPR)*Constants.wheelPerimeter);

        intakeOnCheckpoint = new Translation2d();
        intakeOffCheckpoit = new Translation2d();
        /*
        driveLMaster.configFactoryDefault();
		driveRMaster.configFactoryDefault();
		driveLTwo.configFactoryDefault();
		driveLThree.configFactoryDefault();
		driveRTwo.configFactoryDefault();
        driveRThree.configFactoryDefault();
        */
        /**
		 * Take our extra motor controllers and have them
		 * follow the VictorSPX updated in arcadeDrive 
		 */
        /*
		driveLTwo.follow(driveLMaster);
		driveLThree.follow(driveLMaster);
		driveRTwo.follow(driveRTwo);
		driveRThree.follow(driveRThree);
        */
        /*
        driveLMaster.setInverted(false);
        driveRMaster.setInverted(true);
        driveLTwo.setInverted(InvertType.FollowMaster);
        driveLThree.setInverted(InvertType.FollowMaster);
        driveRTwo.setInverted(InvertType.FollowMaster);
        driveRThree.setInverted(InvertType.FollowMaster);

        differentialDrive.setRightSideInverted(false);
        */
    }

    public void createRamseteManager(Trajectory trajectory){
        if (ramseteManager == null){
            // Paste this variable in
            /*RamseteController disabledRamsete = new RamseteController() {
                @Override
                public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                        double angularVelocityRefRadiansPerSecond) {
                    return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
                }
            };*/

            ramseteManager = new RamseteManager(trajectory,
                                            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),//disabledRamsete
                                            new SimpleMotorFeedforward(Constants.kDriveS, Constants.kDriveV, Constants.kDriveA),
                                            Constants.kDriveKinematics,
                                            new PIDController(Constants.kTrajectoryP, 0, 0),//leftController
                                            new PIDController(Constants.kTrajectoryP, 0, 0));//rightController
            // Reset odometry to the starting pose of the trajectory.
            ramseteManager.initialize();
        }
        else{
            DriverStation.reportError("Ramsete Manager is already assigned!", false);
        }
    }

    public void resetRamsete(){
        ramseteManager = null;
    }

    /**
     * Follow Trajectory
     * @return false if not finished true if finished
     */
    public boolean followTrajectory(){
        if (!ramseteManager.isFinished()){
            double[] motorVolts = ramseteManager.calculate(getPose(), getWheelSpeeds());
            tankDriveVolts(motorVolts[0], motorVolts[1]);
            return false;
        }
        else{
            ramseteManager.end();
            System.out.println("Dereference the ramsete");
            ramseteManager = null; // dereference object for rereference
            return true;
        }
    }

    public void constructOdometry(){
        constructOdometry(getFusedGyroRotation2D());
    }

    public void constructOdometry(Rotation2d gyroRotation){
        odometry = new DifferentialDriveOdometry(gyroRotation);
    }

    public void constructOdometry(Rotation2d gyroRotation, Pose2d startingPos){
        odometry = new DifferentialDriveOdometry(gyroRotation, startingPos);
    }

    public void updateOdometry(){
        updateOdometry(getFusedGyroRotation2D(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }

    /**
     * 
     * @param gyroRotation
     * @param leftDistance !IN METERS!
     * @param rightDistance !IN METERS!
     */
    public void updateOdometry(Rotation2d gyroRotation, double leftDistance, double rightDistance){
        odometry.update(gyroRotation, leftDistance, rightDistance);
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    
    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoder();
        odometry.resetPosition(pose, getFusedGyroRotation2D());
    }

    public double getSpeed(){
        return (leftEncoder.getRate() + rightEncoder.getRate()) / 2; 
    }

    /**
     * Get Gyro Angle From Pigeon
     * @return gyro_angle
     */
    public double getGyroAngle(){
        return pigeon.getFusedHeading();
    }

    /**
     * Only get yaw angle
     */
    public double getYawAngle(){
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0];
    }

    public Rotation2d getYawGyroRotation2D(){
        return Rotation2d.fromDegrees(Math.IEEEremainder(getYawAngle(), 360.0));
    }

    public Rotation2d getFusedGyroRotation2D(){
        return Rotation2d.fromDegrees(Math.IEEEremainder(getGyroAngle(), 360.0));
    }

    public void resetYawAngle(){
        pigeon.setYaw(0);
    }

    /**
     * Reset Gyro Values
     */
    public void gyroReset(){
        pigeon.setYaw(0);
        pigeon.setAccumZAngle(0);
        pigeon.setFusedHeading(0);
    }

    public void resetEncoder(){
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void resetSensors(){
        gyroReset();
        resetEncoder();
    }

    public boolean isPigeonReady(){
        PigeonState state = pigeon.getState();
        return state == PigeonState.Ready;
    }

    /**
     * Robot Drive Using Arcade Drive
     * @param speed
     * @param rotation_speed
     */
    public void robotDrive(double speed, double rotation_speed){
        robotDrive(speed, rotation_speed, 1);
    }

    /**
     * Robot Drive Using Arcade Drive
     * @param speed
     * @param rotation_speed
     * @param sensitivity
     */
    public void robotDrive(double speed, double rotation_speed, double sensivity){
        differentialDrive.arcadeDrive(speed, rotation_speed*sensivity);
    }

    public void curvatureDrive(double speed, double rotation, boolean isQuickTurn){
        differentialDrive.curvatureDrive(speed, rotation, isQuickTurn);
    }

    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotor.setVoltage(leftVolts);
        rightMotor.setVoltage(-rightVolts);
        differentialDrive.feed();
    }

    /**
     * Semi-auto assisted drive with PID and user control. Less feedback more accuracy
     */
    /*public void assistedDrive(){
    }*/

    /**
     * Fully automated drive with PID
     * @param speed
     * @param rotation_angle
     */
    public void PIDDrive(double speed, double rotation_angle){
        double drivePID = pid.calculate(getGyroAngle(), rotation_angle);
        drivePID = MathUtil.clamp(drivePID, -1, 1);
        differentialDrive.curvatureDrive(speed, drivePID, false);
    }

    public void resetPID(){
        pid.reset();
    }

    /**
     * Resets everything resettable
     */
    public void reset(){
    }

    public double simpleTurnPID(double desired_rotation){
        double rotation = 0;
        double kP = SmartDashboard.getNumber("Turn PID", 0.01);
        double minMax = SmartDashboard.getNumber("Min PID", 0.3);
        if (desired_rotation > 1.0){
                rotation = kP*desired_rotation + minMax;
        }
        else if (desired_rotation < 1.0){
                rotation = kP*desired_rotation - minMax;
        }
        return rotation;
    }

    public double turnPID(double desired_rotation){
        double rotation = 0;
        double kP = SmartDashboard.getNumber("Turn PID", 0.1);
        double minMax = 1.57;
        if (desired_rotation > 1.0){ // to the right
                rotation = kP*desired_rotation + minMax;
        }
        else if (desired_rotation < 1.0){ // to the left
                rotation = kP*desired_rotation - minMax;
        }
        tankDriveVolts(rotation, -rotation);
        return rotation;
    }

    /*public double rotationCharacterization(){

    }*/

    /**
     * Stop Driving Robot
     */
    public void stopDrive(){
        differentialDrive.tankDrive(0, 0);
    }
}
