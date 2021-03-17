// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Action;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class TestAction implements Action{
    double _time;
    String _message;
    Timer timer;

    public TestAction(String message, double time){
        timer = new Timer();
        _message = message;
        _time = time;
    }
    @Override
    public void start() {
        timer.reset();
        timer.start();
    }

    @Override
    public void update() {
        System.out.println(_message);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > _time;
    }

    @Override
    public void done() {
    }

}
