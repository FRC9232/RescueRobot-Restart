// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;

/** An example command that uses an example subsystem. */
public class AutonomousClimb extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_DriveTrain;
  private final ArmSubsystem m_Arm;
  private int driveMode; //1-move forward, 2-climb, 3-go down
  private double initialYaw;
  private double pitch;
  //private double climbPositionForFront = -1200;
  private double climbPositionForFront = -1200;
  private double climbPositionForBack = -2400;
 
  private double driveSpeed = 0.6;
  private double kF = 0;
  private double frontArmtargetPosition = 0;
  private double backArmtargetPosition = 0;
  private final double kc = 0.1;
  private boolean armIsGoingHome = false;
  private double downPositionForFront = -1000;
  private boolean stopLevelStarted = false;
  private boolean stoppingDone = false;
  private Timer stoppingTimer = new Timer();

  public AutonomousClimb(DriveTrain driveTrain, ArmSubsystem armSubsystem) {
    m_DriveTrain = driveTrain;
    m_Arm = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
    addRequirements(m_Arm);
    
    driveMode = 1; // 1 yap
    
    initialYaw = m_DriveTrain.gyro.getYaw();
    pitch = m_DriveTrain.gyro.getPitch();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Start to move forward
    frontArmtargetPosition = -510;
    armGoposition();
    if(Constants.Drive.downMode){
      driveMode = 4;
    }
  }

  public void updateGyro(){
    pitch = m_DriveTrain.gyro.getPitch();
  }

  public void armGoposition(){
    if(frontArmtargetPosition != 0){
      double currentPosition = m_Arm.frontArm.getSelectedSensorPosition();
      double error = frontArmtargetPosition - currentPosition;
      //System.out.println(error);
      double maxSpeed = 0.6;
      
      if(kF == 0){
        kF = maxSpeed / error;
        if(error <0){
          kF = kF * -1;
        }else{
          maxSpeed = 0.7;
          kF = maxSpeed / error;
        }
      }
      if(error <= 50 && error >=-50){
        m_Arm.setPercent("front", error*kF+kc);
        //kF=0;
        //frontArmtargetPosition = 0;
      }
      else{
        m_Arm.setPercent("front", error*kF);
      }
    }
  }

  public void backArmGoposition(){
    if(backArmtargetPosition != 0){
      double currentPosition = m_Arm.backArm.getSelectedSensorPosition();
      double error = backArmtargetPosition - currentPosition;
      double maxSpeed = 0.9;

      if(kF == 0){
        kF = maxSpeed / error;
        if(error <0){
          kF = kF * -1;
        }else{
          maxSpeed = 0.9;
          kF = maxSpeed / error;
        }
      }

      if(error <= 50 && error >=-50){
        m_Arm.setPercent("back", error*kF+kc);
        kF=0;
        backArmtargetPosition = 0;
      }
      else{
        m_Arm.setPercent("back", error*kF);
      }
    }
  }

  public void armsGoHome(){
    if(armIsGoingHome){
      m_Arm.setPercent("front", 0.4);
      m_Arm.setPercent("back", 0.3);

      if(m_Arm.limitSwitchBackArm.get()){
        m_Arm.setPercent("back", 0.1);
      }
  
      if(m_Arm.limitSwitchFrontArm.get()){
        m_Arm.setPercent("front", 0.1);
      }
  
      if(m_Arm.limitSwitchBackArm.get() && m_Arm.limitSwitchFrontArm.get()){
        armIsGoingHome = false;
      }
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateGyro();
    armGoposition();
    backArmGoposition();
    armsGoHome();

    
    

    m_DriveTrain.driveMode = driveMode;
    SmartDashboard.putNumber("Step Count", m_DriveTrain.stepCount);
    SmartDashboard.putNumber("Drive mode", driveMode);
    if(driveMode == 1){
      //Get gyro yaw angle while moving forward
      //m_Arm.teleopArcadeDrivePercent(-0.1, "front");
      //m_Arm.teleopArcadeDrivePercent(-0.1, "back");
      double error = m_DriveTrain.gyro.getYaw() - initialYaw;
      double kp = 1/360;
      double turn = 0.0;
      if(error > 0){ //it is going right, turn should be positive
        turn = error * kp;
      }
      else if(error <0){
        turn = -error*kp*driveSpeed;
      }
     
      m_DriveTrain.teleopArcadeDrivePercent(driveSpeed, turn);

      if(pitch >= 5){
        driveMode = 2;
      }
      
    }
    else if(driveMode == 2){
      if(pitch >= 5){
        frontArmtargetPosition = climbPositionForFront;
        backArmtargetPosition = climbPositionForBack;
      }
      else{
        m_DriveTrain.stepCount +=1;
       // driveMode = 3;
       // m_Arm.setPosition(2);
       // m_Arm.setPositionBackArm(0);
       backArmtargetPosition = 1;
       if(m_DriveTrain.stepCount >= 4){
        driveMode = 3; //teleop and go down
        
        
       }
       else{
        driveMode = 1;

       }

        Timer.delay(1);
        armIsGoingHome = true;
      }

    }
    else if(driveMode == 3){
      armIsGoingHome = true;//it should work
      if(!stoppingDone){
      
        if(!stopLevelStarted){
          stoppingTimer.start();
          stopLevelStarted = true;
        }
        else{
          if(stoppingTimer.get() >= 0.7){
           
            m_DriveTrain.teleopArcadeDrivePercent(0.0, 0.0);
            stoppingTimer.stop();
            stoppingDone = true;
            Constants.Drive.downMode = true;
            
          }
        }
      }
    }
    else if(driveMode == 4){
      //go down
      double error = m_DriveTrain.gyro.getYaw() - initialYaw;
      double kp = 1/360;
      double turn = 0.0;
      if(error > 0){ //it is going right, turn should be positive
        turn = error * kp;
      }
      else if(error <0){
        turn = -error*kp*driveSpeed;
      }
     
      m_DriveTrain.teleopArcadeDrivePercent(driveSpeed, turn);

      if(pitch <= -5){

        //front arm to down
        //frontArmtargetPosition = climbPositionForFront;

      }
      else{
        //front arm to home
        //armIsGoingHome = true;
        frontArmtargetPosition = downPositionForFront;
        
      }

    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
