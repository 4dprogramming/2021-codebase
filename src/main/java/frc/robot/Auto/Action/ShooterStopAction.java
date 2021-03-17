/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Auto.Action;

import frc.robot.Subsystems.Shooter;

/**
 * Add your docs here.
 */
public class ShooterStopAction implements Action {

    private Shooter mShooter;
    boolean a = false;
    
    public ShooterStopAction(){
        mShooter = Shooter.getInstance();
    }

    @Override
    public void done() {
        Shooter.autoShooterState.nextState();
    }

    @Override
    public boolean isFinished() {
        return a;
    }

    @Override
    public void start() {
        mShooter.shooterStop();
        mShooter.feederStop();
        a = true;
    }

    @Override
    public void update() {
    }
}
