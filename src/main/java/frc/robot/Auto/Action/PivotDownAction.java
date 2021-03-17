/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Auto.Action;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Intake;

/**
 * Add your docs here.
 */
public class PivotDownAction implements Action {

    Timer timer;
    Intake mIntake;
    double _pivotTime;

    public PivotDownAction(double pivotTime){
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
        mIntake.pivotDown();
    }
}
