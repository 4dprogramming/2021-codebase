// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

/** Add your docs here. */
public class RobotInfo {
    public enum EncoderMode{
        SHOOTER,
        DRIVE
    }
    
    public static Trajectory trajectory;
    public static Field2d fieldSim = new Field2d();
}
