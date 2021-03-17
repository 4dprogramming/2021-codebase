// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;

/** Port of Wpilib's RamseteCommand for Timed Robot. Manages Ramsete Controller */
public class RamseteManager {
    private Timer m_timer = new Timer();
    private boolean m_usePID; // are you using pid
    private Trajectory m_trajectory;
    private RamseteController m_follower;
    private SimpleMotorFeedforward m_feedforward;
    private DifferentialDriveKinematics m_kinematics;
    private PIDController m_leftController;
    private PIDController m_rightController;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory. PID
     * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
     * units of volts.
     *
     * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
     * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param controller The RAMSETE controller used to follow the trajectory.
     * @param feedforward The feedforward to use for the drive.
     * @param kinematics The kinematics for the robot drivetrain.
     * @param leftController The PIDController for the left side of the robot drive.
     * @param rightController The PIDController for the right side of the robot drive.
     */
    @SuppressWarnings("PMD.ExcessiveParameterList")
    public RamseteManager(Trajectory trajectory, RamseteController controller, SimpleMotorFeedforward feedforward,
                          DifferentialDriveKinematics kinematics, PIDController leftController, PIDController rightController) {
        m_trajectory = trajectory;
        m_follower = controller;
        m_feedforward = feedforward;
        m_kinematics = kinematics;
        m_leftController = leftController;
        m_rightController = rightController;
        m_usePID = true;
    }

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds from
     * the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
     * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param controller The RAMSETE controller used to follow the trajectory.
     * @param kinematics The kinematics for the robot drivetrain.
     */
    @SuppressWarnings("PMD.ExcessiveParameterList")
    public RamseteManager(Trajectory trajectory, RamseteController controller, DifferentialDriveKinematics kinematics) {
        m_trajectory = trajectory;
        m_follower = controller;
        m_kinematics = kinematics;
        m_usePID = false;
    }

    public void initialize() {
        m_prevTime = -1;
        State initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                                                  initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
        if (m_usePID) {
          m_leftController.reset();
          m_rightController.reset();
        }
      }
    
    /**
     * Calculate the motor volts using ramsete controller. If no PID is during  construction of the class values won't
     * be processed by PID and user should modify it.
     * @param currentPose Current Position of robot. Use Odometry for this
     * @param wheelSpeeds Current wheel speed of robot.
     * @return Returns a double array for motor volts. [0] is for left, [1] is for right motor
     */
    public double[] calculate(Pose2d currentPose, DifferentialDriveWheelSpeeds wheelSpeeds) {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        if (m_prevTime < 0) {
            m_prevTime = curTime;
            double[] motorVolts = {0.0 , 0.0};
            return motorVolts;
        }

        DifferentialDriveWheelSpeeds targetWheelSpeeds = m_kinematics.toWheelSpeeds(m_follower.calculate(currentPose, 
                                                                                                         m_trajectory.sample(curTime)));

        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if (m_usePID) {
            double leftFeedforward = m_feedforward.calculate(leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);
            double rightFeedforward = m_feedforward.calculate(rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

            leftOutput = leftFeedforward + m_leftController.calculate(wheelSpeeds.leftMetersPerSecond, leftSpeedSetpoint);
            rightOutput = rightFeedforward + m_rightController.calculate( wheelSpeeds.rightMetersPerSecond, rightSpeedSetpoint);
        } 
        else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        double[] motorVolts = {leftOutput, rightOutput};
        m_prevSpeeds = targetWheelSpeeds;
        m_prevTime = curTime;
        return motorVolts;
    }

    public void end() {
        m_timer.stop();
    }

    public boolean isFinished(){
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}