/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Auto.Modes;

import java.util.Arrays;

import frc.robot.Auto.AutoModeEndedException;
import frc.robot.Auto.Action.*;

/**
 * Add your docs here.
 */
public class SimpAuto extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SeriesAction(Arrays.asList(
                        new ParallelAction(Arrays.asList(
                            new BlindSpeedUpAction(0.78, 0.78),
                            new SeriesAction(Arrays.asList(
                                new SeekTargetAndAimAction(),
                                new ParallelAction(Arrays.asList(
                                    new BlindShootAction(0.15, 1), 
                                    new CreateFieldRelativeTrajectory()
                                ))
                            ))
                        )),
            new ParallelAction(new FollowTrajectoryAction(), new TrajectoryIntakeAction(2)),
            new ParallelAction(new BlindSpeedUpAction(0.78, 0.78),
                                new SeriesAction(new SeekTargetAndAimAction(),
                                new BlindShootAction(0.15, 1)
                                )),
            new StopAction()
        )));
    }
}
