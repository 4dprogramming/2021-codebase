// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public final class Paths {
    public static final Pose2d slalomStart = new Pose2d(0.72, -3.6, new Rotation2d());
    public static final Pose2d slalomEnd = new Pose2d(0.5499212598, -2.044724409, Rotation2d.fromDegrees(-180));
    public static final List<Translation2d> slalomWaypoints = List.of(new Translation2d(1.968188976, -3.454488189),
                                                new Translation2d(2.992440945, -2.087244094),
                                                new Translation2d(4.419212598, -1.685669291),
                                                new Translation2d(6.042519685, -2.429291339),
                                                new Translation2d(6.588661417, -3.419527559),
                                                new Translation2d(7.510866142, -3.693543307),
                                                new Translation2d(8.262992126, -2.873385827),
                                                new Translation2d(7.681889764, -2.010708661),
                                                new Translation2d(6.742677165, -2.164724409),
                                                new Translation2d(6.298582677, -2.907401575),
                                                new Translation2d(5.623937008, -3.702047244),
                                                new Translation2d(4.163149606, -3.787086614),
                                                new Translation2d(2.685354331, -3.428031496),
                                                new Translation2d(1.57511811, -2.215748031));

    public static final Pose2d manuelSlalomStart = new Pose2d(0.72, -3.60, new Rotation2d());
    public static final Pose2d manuelSlalomEnd = new Pose2d(0.72, -2.16, Rotation2d.fromDegrees(-180));
    public static final List<Translation2d> manuelSlalomWaypoints = List.of(new Translation2d(2.16, -3.60), //3,-5
                                                                            new Translation2d(2.16, -2.16), //3,-3
                                                                            new Translation2d(6.39, -2.16), //9,-3
                                                                            new Translation2d(6.39, -3.60), //9,-5
                                                                            new Translation2d(7.92, -3.60), //11,-5
                                                                            new Translation2d(7.92, -2.16), //11,-3
                                                                            new Translation2d(6.39, -2.16), //9,-3
                                                                            new Translation2d(6.39, -3.60), //9,-5
                                                                            new Translation2d(2.16, -3.60), //3,-5
                                                                            new Translation2d(2.16, -2.16) //3,-3
    );
                                        
    public static final List<Pose2d> manuelClampedSlamom = List.of(new Pose2d(0.72, -3.60, new Rotation2d()),
                                                                   new Pose2d(2.16, -3.60, Rotation2d.fromDegrees(90)),
                                                                   new Pose2d(2.16, -2.16, new Rotation2d()),
                                                                   new Pose2d(6.39, -2.16, Rotation2d.fromDegrees(-90)),
                                                                   new Pose2d(6.39, -3.60, new Rotation2d()),
                                                                   new Pose2d(7.92, -3.60, Rotation2d.fromDegrees(90)),
                                                                   new Pose2d(7.92, -2.16, Rotation2d.fromDegrees(-180)),
                                                                   new Pose2d(6.39, -2.16, Rotation2d.fromDegrees(-90)),
                                                                   new Pose2d(6.39, -3.60, Rotation2d.fromDegrees(-180)),
                                                                   new Pose2d(2.16, -3.60, Rotation2d.fromDegrees(90)),
                                                                   new Pose2d(2.16, -2.16, Rotation2d.fromDegrees(-180)),
                                                                   new Pose2d(0.72, -2.16, Rotation2d.fromDegrees(-180))
    );
}
