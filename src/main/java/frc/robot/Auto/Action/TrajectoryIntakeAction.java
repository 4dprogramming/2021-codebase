// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto.Action;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Utils;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

/** Add your docs here. */
public class TrajectoryIntakeAction implements Action {
    enum PivotCommand{
        HALT,
        DESCEND,
        ASCEND
    }

    enum IntakeCommand{
        OFF,
        IN,
        REVERSE
    }

    Shooter mShooter;
    Drive mDrive;
    Intake mIntake;
    Timer timer;
    double startTime;
    double _pivotTime;
    PivotCommand pivotCommand;
    IntakeCommand intakeCommand;
    boolean isFinished;
    boolean canShoot;
        
    public TrajectoryIntakeAction(double pivotTime){
        mDrive = Drive.getInstance();
        mIntake = Intake.getInstance();
        mShooter = Shooter.getInstance();
        timer = new Timer();
        _pivotTime = pivotTime;
    }
    @Override
    public void start() {
        timer.reset();
        timer.start();
        startTime = -1;
        pivotCommand = PivotCommand.HALT;
        intakeCommand = IntakeCommand.OFF;
        isFinished = false;
        canShoot = false;
    }

    @Override
    public void update() {
        Pose2d currentPos = mDrive.getPose();
        if (startTime == -1){
            startTime = timer.get();
            pivotCommand = PivotCommand.DESCEND;
        }
        if (Utils.isRobotInPosition(currentPos, Drive.intakeOnCheckpoint, new Translation2d(0.5, 0.5))){
            intakeCommand = IntakeCommand.IN; 
        }
        else if (Utils.isRobotInPosition(currentPos, Drive.intakeOffCheckpoit.plus(new Translation2d(-0.75, 0)), new Translation2d(0.1, 0.1))){
            // pivot and intake stop action
            if (startTime == -2){
                startTime = timer.get();
                pivotCommand = PivotCommand.ASCEND;
            }
            intakeCommand = IntakeCommand.OFF;
            canShoot = true;
        }

        if (pivotCommand == PivotCommand.DESCEND){
            if(timer.get() - startTime < _pivotTime){
                pivotCommand = PivotCommand.DESCEND;
            }
            else{
                pivotCommand = PivotCommand.HALT;
                startTime = -2;
            }
        }
        else if(pivotCommand == PivotCommand.ASCEND){
            if(timer.get() - startTime < _pivotTime){
                pivotCommand = PivotCommand.ASCEND;
            }
            else{
                pivotCommand = PivotCommand.HALT;
                isFinished = true;
            }
        }

        if (canShoot){
            mShooter.setShooterMotorSpeed(0.75);
            mShooter.setAccMotorSpeed(0.75);
        }
        switch (pivotCommand){
            case HALT:
                mIntake.pivotStall();
                break;
            case DESCEND:
                mIntake.pivotDown();
                break;
            case ASCEND:
                mIntake.pivotUp();
                break;
        }

        switch (intakeCommand){
            case OFF:
                mIntake.intakeStop();
                break;
            case IN:
                mIntake.intakeOn();
                break;
            case REVERSE:
                mIntake.intakeReverse();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void done() {
        
    }

}
