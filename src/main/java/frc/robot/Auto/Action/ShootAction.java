// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Shooter;

/** Add your docs here. */
public class ShootAction implements Action {

    Shooter mShooter;
    double _wantedRPM;
    Timer timer; // TODO now done by timer but in the future do this by detecting how many balls shooted

    public ShootAction(double wantedRPM){
        mShooter = Shooter.getInstance();
        _wantedRPM = wantedRPM;
    }

    @Override
    public void done() {
        Shooter.autoShooterState.nextState();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 5;
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
        Shooter.autoShooterState.nextState();
    }

    @Override
    public void update() {
        mShooter.shoot(_wantedRPM);
    }
    
}
