/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Auto.Action;

import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Shooter;

/**
 * Add your docs here.
 */
public class ResetSensorsAction implements Action {

    private boolean isReset = false;
    private Drive mDrive;
    private Shooter mShooter;

    @Override
    public void done() {
    }

    @Override
    public boolean isFinished() {
        return isReset;
    }

    @Override
    public void start() {
        mDrive = Drive.getInstance();
        mShooter = Shooter.getInstance();
        mDrive.resetSensors();
        mShooter.reset();
        isReset = true;
    }

    @Override
    public void update() {
    }
}
