// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Constants;

/** Subsystem for climb */
public class Climb {
    private static Climb mInstance = new Climb();

    private VictorSP climbMotor;

    public static Climb getInstance(){
        return mInstance;
    }

    private Climb(){
        climbMotor = new VictorSP(Constants.climberMotorPort);
        climbMotor.setInverted(true);
    }

    public void releaseClimber(){
        climbMotor.set(0.5);
    }

    public void climb(){
        climbMotor.set(-0.7);
    }

    public void hang(){
        climbMotor.set(-0.3);
    }

    public void stopClimbMotor(){
        climbMotor.set(0);
    }
}
