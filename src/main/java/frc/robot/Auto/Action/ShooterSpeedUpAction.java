// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Action;

import frc.robot.Utils;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Shooter.AutoShooterStates;

/** Add your docs here. */
public class ShooterSpeedUpAction implements Action {

    Shooter mShooter;
    double _wantedRPM;

    public ShooterSpeedUpAction(double wantedRPM){
        _wantedRPM = wantedRPM;
        mShooter = Shooter.getInstance();
    }
    @Override
    public void done() {
    }

    @Override
    public boolean isFinished() {
        return Shooter.autoShooterState == AutoShooterStates.SHOOTED;
    }

    @Override
    public void start() {
        Shooter.autoShooterState.nextState();
    }

    @Override
    public void update() {
        mShooter.shooterSpeedUp(_wantedRPM);
        if (Utils.tolerance(mShooter.getShooterRPM(), _wantedRPM, 50) && (Shooter.autoShooterState == AutoShooterStates.SPEEDING_UP)){
            Shooter.autoShooterState.nextState();
        }
    }
    
}
