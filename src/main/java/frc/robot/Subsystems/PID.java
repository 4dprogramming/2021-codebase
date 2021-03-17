package frc.robot.Subsystems;
import frc.robot.Utils;

public class PID {
    private static PID mInstance = new PID();

    public static PID getInstance(){
        return mInstance;
    }

    public static final double kTurnP = 0;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;
    public static final double turnOutMax = 0;
    public static final double turnOutMin = 0;

    public double TurnError;
    public double TurnIntegral ;
    public double TurnDerivative; 
    public double TurnPreviousError; 
    public double TurnReset; 

    public double turnPID(double wantedAngle, double currentAngle){
    double TurnError = wantedAngle-currentAngle;
    TurnIntegral += TurnError*kTurnI*0.02;
    double TurnDerivative = TurnError-TurnPreviousError;
    TurnIntegral = Utils.applyDeadband(TurnIntegral, 0, 0);
    double TurnReset = TurnIntegral+(TurnError*kTurnP)+(TurnDerivative*kTurnD);
    TurnReset = Utils.applyDeadband(TurnReset, 0,0 );

        TurnPreviousError = TurnError;

        return TurnReset;

    }
    
    public void TurnPidReset(){
        TurnError = 0;
        TurnIntegral = 0;
        TurnDerivative = 0;
    }
}


