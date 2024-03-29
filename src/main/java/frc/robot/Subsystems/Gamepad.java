// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Add your docs here.
 */
public class Gamepad {

    public enum GamepadMode{
        NOSUPPORT,
        ARBITRARYFEEDFORWARD,
        PIDSUPPORT
    }

    private static Gamepad mInstance = new Gamepad();
    
    public static Gamepad getInstance(){
        return mInstance;
    }

    public XboxController gamepad;

    public Gamepad(){
        gamepad = new XboxController(0);
    }

    public double getForward(){
        return gamepad.getRawAxis(3);
    }

    public double getReverse(){
        return gamepad.getRawAxis(2);
    }

    public double getSteering(){
        return gamepad.getRawAxis(0);
    }

    public double getSensetiveSteering(){
        return gamepad.getRawAxis(4);
    }

    public boolean getStartShooting(){
        return gamepad.getRawButton(3);
    }

    public boolean getIntakeGamepad(){
        return gamepad.getRawButton(1);
    }

    public boolean getReverseIntakeGamepad(){
        return gamepad.getRawButton(2);
    }

    public void forceFeedback(double speed, double rotation){
        double leftRotation;
        double rightRotation;
        if (rotation < 0){
            leftRotation = 0.5 * (Math.abs(rotation) + speed); 
            rightRotation = 0.5 * (Math.abs(speed));
        }
        else{
            leftRotation = 0.5 * Math.abs(speed);
            rightRotation = 0.5 * (Math.abs(rotation) + speed);
        }
        gamepad.setRumble(RumbleType.kLeftRumble, leftRotation);
        gamepad.setRumble(RumbleType.kRightRumble, rightRotation);
    }
}