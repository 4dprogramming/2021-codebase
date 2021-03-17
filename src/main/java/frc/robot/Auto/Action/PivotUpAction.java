// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Intake;

/** Add your docs here. */
public class PivotUpAction implements Action {
    Timer timer;
    Intake mIntake;
    double _pivotTime;

    public PivotUpAction(double pivotTime){
        _pivotTime = pivotTime;
    }


    @Override
    public void done() {
        mIntake.pivotStall();        
    }

    @Override
    public boolean isFinished() {
        return timer.get() > _pivotTime;
    }

    @Override
    public void start() {
        mIntake = Intake.getInstance();
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        mIntake.pivotUp();
    }
}
