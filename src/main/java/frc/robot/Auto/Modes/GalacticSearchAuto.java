// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Modes;

import frc.robot.Auto.AutoModeEndedException;
import frc.robot.Auto.Action.CreatePathWeaverTrajectory;
import frc.robot.Auto.Action.FollowTrajectoryAction;
import frc.robot.Auto.Action.SeriesAction;
import frc.robot.Auto.Action.CreateAndFollowMultipleTrajectory.PathType;

/** Add your docs here. */
public class GalacticSearchAuto extends AutoModeBase {

    PathType _path;

    public GalacticSearchAuto(PathType path){
        _path = path;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SeriesAction(new CreatePathWeaverTrajectory(_path),
                                   new FollowTrajectoryAction()
        ));
    }

}
