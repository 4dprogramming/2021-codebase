/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * Add your docs here.
 */
public final class Constants {
    //Motor Ports
    public static final int driveOneLeftMotorPort = 1; //3 VictorSPX not tested
    public static final int driveTwoLeftMotorPort = 2; //3 VictorSPX not tested
    public static final int driveThreeLeftMotorPort = 3; //3 VictorSPX not tested
    public static final int driveOneRightMotorPort = 4; //3 VictorSPX not tested
    public static final int driveTwoRightMotorPort = 5; //3 VictorSPX not tested
    public static final int driveThreeRightMotorPort = 6; //3 VictorSPX not tested

    public static final int conveyorMotorPort = 6; //1 ok
    
    public static final int pivotMotorPort = 3; //1 ok
    public static final int centerRightMotorPort = 4; //1 ok
    public static final int centerLeftMotorPort = 2; //1 ok
    public static final int intakeMotorPort = 5; //1 ok
    public static final int climberMotorPort = 7; // 1 ok
   
    public static final int feederMotorPort = 8; //1 ok
    public static final int acceleratorWheelMotorPort = 1; //1 ok
    public static final int shooterWheelMotorPort = 0; //2 ok

    // Pigeon Port
    public static final int pigeonPort = 8;

    //Encoder Ports
    public static final int shooterEncPort1 = 12;//
    public static final int shooterEncPort2 = 13; //
    
    public static final int accEncPort1 = 10; // ok enc2
    public static final int accEncPort2 = 11; // ok

    public static final int driveLeftEncPort1 = 16; //
    public static final int driveLeftEncPort2 = 17; //

    public static final int driveRightEncPort1 = 14; //
    public static final int driveRightEncPort2 = 15; //

    // Dead Zone
    public static final double speedDeadZone = 0.25;
    public static final double rotationDeadZone = 0.3; // not tested


    //Encoder Directions
    public static final boolean shooterEncDirection = true;
    public static final boolean accEncDirection = true;

    //Distance per pulse values
    public static final double shooterEncDistancePerPulse = 0.00277777777;
    public static final double accEncDistancePerPulse = 0.00277777777;
    public static final double wheelPerimeter = 0.31918581360472299303;
    public static final double encoderCPR = 360;

    //PID Constants
    public static final double kShooterP = 0.0314;
    public static final double kShooterI = 0;
    public static final double kShooterD = 0.00001;

    public static final double kAccP = 0.0144;
    public static final double kAccI = 0;
    public static final double kAccD = 0.00001;

    public static final double kDriveP = 6.61;
    public static final double kDriveI = 0;
    public static final double kDriveD = 2.86;

    //Position PID 
    //P is 5.59
    //D is 2.64

    //Feedforward Constants
    public static final double kShooterS = 1.06;
    public static final double kShooterV = 0.127;
    public static final double kShooterA = 0.00157;

    public static final double kAccS = 0.772;
    public static final double kAccV = 0.102;
    public static final double kAccA = 0.000977;

    public static final double kDriveS = 1.26;//1.11;//1.32; 
    public static final double kDriveV = 3.04;//3.14;//3.04;//1.52;
    public static final double kDriveA = 0.657;//0.552;//0.289;//0.185;

    // Simple Turn PID
    public static final double kSimpleTurnP = 0.015;
    public static final double kSimpleTurnMin = 0.3;

    public static final double turnOutMax = 1;
    public static final double turnOutMin = -1;

    // Vision
    public static final double limelightHeightFromGround = 0.57; // might do cm for not using float point math
    public static final double limelightAngle = 34; //56 //34
    public static final double visionTargetHeightFromGround = 2.29;
    public static final double limelightBlobAreaEstimation = 0;

    // Drive Constants
    public static final double kTrackWidth = 0.574; // 0.5950743566697735; // Meters
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);
    public static final double kMaxSpeed = 3.2; // MetersPerSecond 3.2
    public static final double kMaxAcceleration = 27; // MetersPerSecondSquared 27
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kTrajectoryP = 2.32;//2.24;

    // Shooter Constants
    public static final double kShooterMaxPower = 0;
    public static final double kFeederMaxPower = 0;
    public static final double kAcceleratorMaxPower = 0;
    public static final double kShootingAngle = 0;

    // Field Constants
    public static final Translation2d kVisionTargetPos = new Translation2d(0, -2.5);
    public static final double initiationLineX = 3.1;
    public static final Translation2d intakeStartPoint = new Translation2d(5, -0.7);
    public static final Translation2d intakeEndPoint = new Translation2d(7.4, -0.7);
    public static final Translation2d falseIntakeStartPoint = new Translation2d(initiationLineX, -0.9);
    public static final Translation2d falseIntakeEndPoint = new Translation2d(initiationLineX + 1.35, -0.9);
    public static final Translation2d trajectoryEndPoint = new Translation2d(initiationLineX, kVisionTargetPos.getY());
    public static final Rotation2d endingRotation = new Rotation2d();
    public static final Pose2d glassOrigin = new Pose2d(0, -4.32, new Rotation2d()); // !beware! rotation 2d here need to change if we decide to start our odometry from a spesific angle
    public static final Pose2d pwOrigin = new Pose2d(0, -4.32, new Rotation2d());

    // Gamepad Constants
    public static final double kSensivity = 0.75;


    /*
    PATHWEAVER VALUES
    0  (0.762, -3.81)   (0.72, -3.6 )
    1  (2.083, -3.656)  (1.968188976 , -3.454488189)
    2  (3.167, -2.209)  (2.992440945 , -2.087244094)
    3  (4.677, -1.784)  (4.419212598 , −1.685669291)
    4  (6.395, -2.571)  (6.042519685 , -2.429291339)
    5  (6.973, -3.619)  (6.588661417 , -3.419527559)
    6  (7.949, -3.909)  (7.510866142 , -3.693543307)
    7  (8.745, -3.041)  (8.262992126 , -2.873385827)
    8  (8.13, -2.128 )  (7.681889764 , -2.010708661)
    9  (7.136, -2.291)  (6.742677165 , -2.164724409)
    10 (6.666, -3.077)  (6.298582677 , −2.907401575)
    11 (5.952, -3.918)  (5.623937008 , -3.702047244)
    12 (4.406, -4.008)  (4.163149606 , -3.787086614)
    13 (2.842, -3.628)  (2.685354331 , -3.428031496)
    14 (1.667, -2.345)  (1.57511811 , −2.215748031)
    15 (0.582, -2.164)  (0.5499212598 , -2.044724409)
    
    */
}
