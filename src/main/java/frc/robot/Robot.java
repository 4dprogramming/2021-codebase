// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.AutoModeExecutor;
import frc.robot.Auto.Action.CreateAndFollowMultipleTrajectory.PathType;
import frc.robot.Auto.Modes.AutoNavAuto;
import frc.robot.Auto.Modes.GalacticSearchAuto;
import frc.robot.Auto.Modes.SimpAuto;
import frc.robot.Subsystems.Climb;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Drive;
import frc.robot.Subsystems.DrivePanel;
import frc.robot.Subsystems.Gamepad;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kBarrelRacingAuto = "Barrel Racing";
  private static final String kBounceAuto = "Bounce Path";
  private static final String kGalacticAAuto = "Galactic A Racing";
  private static final String kGalacticBAuto = "Galactic B Racing";
  private static final String kSlolomAuto = "Slolom";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private static final int kProcessingMode = 1;
  private static final int kDrivingMode = 2;
  private int mSelectedMode;
  private final SendableChooser<Integer> m_cameraChooser = new SendableChooser<>();

  private Conveyor mConveyor;
  private Drive mDrive;
  private DrivePanel mDrivePanel;
  private Gamepad mGamepad;
  private Intake mIntake;
  private Shooter mShooter;
  private Climb mClimb;
  private Vision mVision;
  private AutoModeExecutor ame;
  private double wantedRPM = 5000;
  private double turnPID = 0.09;
  private double maxSpeed = 0;
  private double maxAcc = 0;
  private double prevTime = 0;
  private double prevLeftSpeed = 0;
  private double prevRightSpeed = 0;
  private double prevLeftDistance = 0;
  private double prevRightDistance = 0;
  private Timer timer;
  private boolean shooterPressed;
  double a = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.setDefaultOption("Slolom", kSlolomAuto);
    m_chooser.addOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Bounce Path", kBounceAuto);
    m_chooser.addOption("Barrel Racing", kBarrelRacingAuto);
    //m_chooser.addOption("Slolom", kSlolomAuto);
    m_chooser.addOption("Galactic A Racing", kGalacticAAuto);
    m_chooser.addOption("Galactic B Racing", kGalacticBAuto);

    SmartDashboard.putData("Auto choices", m_chooser);

    m_cameraChooser.setDefaultOption("Processing Mode", kProcessingMode);
    m_cameraChooser.addOption("Driving Mode", kDrivingMode);
    
    SmartDashboard.putData("Limelight Mode", m_cameraChooser);

    mConveyor = Conveyor.getInstance();
    mDrive = Drive.getInstance();
    mDrivePanel = DrivePanel.getInstance();
    mGamepad = Gamepad.getInstance();
    mIntake = Intake.getInstance();
    mShooter = Shooter.getInstance();
    mClimb = Climb.getInstance();
    mVision = Vision.getInstance();
    ame = new AutoModeExecutor();
    mVision.setLedMode(0);
    mShooter.resetSensors();
    mShooter.resetPID();
    mDrive.resetSensors();
    SmartDashboard.putNumber("Wanted RPM", wantedRPM);
    SmartDashboard.putNumber("Turn PID", turnPID);
    timer = new Timer();
    timer.reset();
    timer.start();
    prevTime = timer.get();
    shooterPressed = false;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*SmartDashboard.putNumber("Shooter RPM", mShooter.getShooterRPM());
    SmartDashboard.putNumber("Accelerator RPM", mShooter.getAccRPM());
    SmartDashboard.putNumber("Shooter Rate", mShooter.shooterEnc.getRate());
    SmartDashboard.putNumber("Accelerator Rate", mShooter.accEnc.getRate());*/
    mSelectedMode = m_cameraChooser.getSelected();
    switch (mSelectedMode){
      case kProcessingMode:
        mVision.setCameraMode(false);
        break;
      case kDrivingMode:
        mVision.setCameraMode(true);
        break;
    }
    wantedRPM = SmartDashboard.getNumber("Wanted RPM", wantedRPM);
    SmartDashboard.putNumber("Wanted RPM", wantedRPM);
    SmartDashboard.putNumber("Shooter RPM", mShooter.getShooterRPM());
    SmartDashboard.putNumber("Acc RPM", mShooter.getAccRPM());
    SmartDashboard.putNumber("Shooter Rate", mShooter.shooterEnc.getRate());
    SmartDashboard.putNumber("Acc Rate", mShooter.accEnc.getRate());
    SmartDashboard.putBoolean("Is Ready For Shoot", mShooter.isReadyForShoot(wantedRPM/60));
    SmartDashboard.putNumber("Drive Right Encoder", mDrive.rightEncoder.getDistance());
    SmartDashboard.putNumber("Drive Left Encoder", mDrive.leftEncoder.getDistance());
    SmartDashboard.putNumber("Gyro", mDrive.getGyroAngle());
    SmartDashboard.putNumber("Heading", mDrive.getFusedGyroRotation2D().getDegrees());
    double[] accAngles = new double[3];
    mDrive.pigeon.getAccelerometerAngles(accAngles);
    SmartDashboard.putNumber("Pigeon Acc X", accAngles[0]);
    SmartDashboard.putNumber("Pigeon Acc Y", accAngles[1]);
    SmartDashboard.putNumber("Pigeon Acc Z", accAngles[2]);
    double leftDistance = mDrive.leftEncoder.getDistance();
    double rightDistance =  mDrive.rightEncoder.getDistance();
    double curTime = timer.get();
    double dT = curTime - prevTime;
    double leftDiff = leftDistance - prevLeftDistance;
    double rightDiff = rightDistance - prevRightDistance;
    double leftSpeed = leftDiff/dT;
    double rightSpeed = rightDiff/dT;
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
    ChassisSpeeds chassisSpeeds = Constants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);
    double currentSpeed = chassisSpeeds.vxMetersPerSecond;
    //double currentSpeed = (leftSpeed + rightSpeed) / 2;
    double currentAcc = 0;
    double leftAcc = (leftSpeed - prevLeftSpeed) / dT;
    double rightAcc = (rightSpeed - prevRightSpeed) / dT;
    if (leftDiff > 0 && rightDiff > 0){
      currentAcc = leftAcc + rightAcc;
    }
    else{
      currentAcc = 0;
    }
    prevTime = curTime;
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);
    SmartDashboard.putNumber("Speed", currentSpeed);
    SmartDashboard.putNumber("Acceleration", currentAcc);
    if (currentSpeed > maxSpeed){
      maxSpeed = currentSpeed;
    }
    if (currentAcc > maxAcc){
      maxAcc = currentAcc;
    }
    SmartDashboard.putNumber("MAX Speed", maxSpeed);
    SmartDashboard.putNumber("MAX Acceleration", maxAcc);
    prevLeftSpeed = leftSpeed;
    prevRightSpeed = rightSpeed;
    prevLeftDistance = leftDistance;
    prevRightDistance = rightDistance;
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    switch (m_autoSelected) {
      case kDefaultAuto:
        ame.setAutoMode(new SimpAuto());
        break;
      case kBounceAuto:
        ame.setAutoMode(new AutoNavAuto(PathType.BOUNCE));
        break;
      case kSlolomAuto:
        ame.setAutoMode(new AutoNavAuto(PathType.SLOLOM));
        break;
      case kBarrelRacingAuto:
        ame.setAutoMode(new AutoNavAuto(PathType.BARRELRACING));
        break;
      case kGalacticAAuto:
        ame.setAutoMode(new GalacticSearchAuto(PathType.GALACTICSEARCHA));
        break;
      case kGalacticBAuto:
        ame.setAutoMode(new GalacticSearchAuto(PathType.GALACTICSEARCHB));
        break;
      default:
        ame.setAutoMode(new SimpAuto());
        break;
    }
    System.out.println("Auto selected: " + m_autoSelected);
    mShooter.resetSensors();
    mShooter.resetPID();
    mDrive.resetSensors();
    ame.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*switch (m_autoSelected) {
      case kDefaultAuto:
        break;
      case kBounceAuto:
        break;
      case kSlolomAuto:
        break;
      case kBarrelRacingAuto:
        break;
      case kGalacticAAuto:
        break;
      case kGalacticBAuto:
        break;
      default:
        // Put default auto code here
        break;
    }*/
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    mShooter.resetSensors();
    mShooter.resetPID();
    mDrive.constructOdometry(mDrive.getFusedGyroRotation2D());
    if (ame != null){
      ame.stop();
      ame.reset();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Teleop: Robot drive
    double speed = mGamepad.getForward() - mGamepad.getReverse();
    double rotation;
    mDrive.robotDrive(speed, mGamepad.getSteering());
    if (Math.abs(mGamepad.getSensetiveSteering()) > 0.1){
      rotation = mGamepad.getSensetiveSteering() * 1;
    }
    else{
      rotation = mGamepad.getSteering() * 0.75;
    }
    //speed = Utils.map(speed, 0, 1, Constants.speedDeadZone, 1);
    /*rotation = Utils.map(rotation, 0, 1, Constants.rotationDeadZone, 1);*/
    mDrive.robotDrive(speed, rotation, 1);
    /*mDrive.updateOdometry();
    NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
    var translation = mDrive.odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());*/
    mGamepad.forceFeedback(speed, rotation);
    // Teleop: Pivot
    if(mDrivePanel.pivotDown()){
      mIntake.pivotDown();
    }
    else if(mDrivePanel.pivotUp()){
      mIntake.pivotUp();
    }
    else{
      mIntake.pivotStall();
    }
    // Teleop: Gamepad Intake
    if(mDrivePanel.intakeIn() || mGamepad.getIntakeGamepad()){
      mIntake.intakeOn();
    }
    else if(mGamepad.getReverseIntakeGamepad()){
      mIntake.intakeReverse();
    }
    else{
      // If manual controls is not commanding to intake
      boolean isNotIntakeUsed = false;
      //Teleop: Manual Left Roller
      if(mDrivePanel.leftRoller()){
        mIntake.leftRoller();
      }
      else if(mDrivePanel.leftRollerReverse()){
          mIntake.leftRollerReverse();
      }
      else{
          isNotIntakeUsed = true;
          mIntake.stopLeftRoller();
      }
      //Teleop: Manual Center Roller
      if(mDrivePanel.centerRoller()){
        mIntake.centerRoller();
      }
      else if(mDrivePanel.centerRollerReverse()){
          mIntake.centerRollerReverse();
      }
      else{
          isNotIntakeUsed = true;
          mIntake.stopCenterRoller();
      }
      //Teleop: Manual Right Roller
      if(mDrivePanel.rightRoller()){
          mIntake.rightRollerReverse();
      }
      else if(mDrivePanel.rightRollerReverse()){
          mIntake.rightRoller();
      }
      else{
          isNotIntakeUsed = true;
          mIntake.stopRightRoller();
      }
      // If intake is not used
      if(isNotIntakeUsed){
        mIntake.intakeStop();
      }
    }

    /*if(mDrivePanel.shooterSpeedUp()){
        mShooter.blindSpeedUp(1);
    }
    else{
        mShooter.shooterStop();
    }*/
    //Teleop: Shoot | Uses feeder and accelator
    /*if(mGamepad.getStartShooting()){
        mShooter.blindShoot();;
    }
    else{
        mShooter.feederOff();
    }*/
    // Teleop: Shooter Speed Up | Uses Shooter and Accelerator Wheel
    if (mDrivePanel.isShooterSpeedUpPressed()){
      shooterPressed = !shooterPressed;
    }

    if (shooterPressed){
      mShooter.shooterSpeedUp(wantedRPM);
    }
    else{
        mShooter.shooterStop();
    }
    /*if(mDrivePanel.shooterSpeedUp()){
        mShooter.shooterSpeedUp(wantedRPM);
    }
    else{
        mShooter.shooterStop();
    }*/
    //Teleop: Shoot | Uses feeder and accelator
    if(mGamepad.getStartShooting()){
        mShooter.shoot(wantedRPM);
    }
    else{
        mShooter.feederOff();
    }

    if (mDrivePanel.climberUp()){
      mClimb.releaseClimber();
    }
    else if (mDrivePanel.climberDown()){
      mClimb.climb();
    }
    else if (mDrivePanel.climbHalt()){
      mClimb.hang();
    }
    else{
      mClimb.stopClimbMotor();
    }

    if (mDrivePanel.resetGyro()){
      mShooter.resetSensors();
      mShooter.resetPID();
      mDrive.resetSensors();
      maxSpeed = 0;
      maxAcc = 0;
      prevLeftSpeed = 0;
      prevRightSpeed = 0;
      mDrive.resetOdometry(new Pose2d());
    }

    if (mDrivePanel.autoAim()){
      if (mDrivePanel.autoAim()){
        double[] visionInfo = mVision.getInfo();
        if (visionInfo[0] > 0){
          
          if (Utils.tolerance(visionInfo[1], 0, 1)){
            double distance = mVision.estimateDistanceFromAngle(visionInfo[2]);
            System.out.println("Distance is " + distance + "m");
          }
          else{
            double arcadeRotation = mDrive.turnPID(visionInfo[1]);
            System.out.println("Arcade is " + arcadeRotation);
          }
        }
      }
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    mGamepad.forceFeedback(0, 0);
    if (ame != null){
      ame.stop();
      ame.reset();
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (Math.abs(mDrive.leftEncoder.getDistance()) + Math.abs(mDrive.rightEncoder.getDistance()) > 0.01){
      System.out.println("Ks for arcade is " + a);
    }
    else{
      a += 0.01;
      mDrive.robotDrive(a, 0);;
    }
  }
}
