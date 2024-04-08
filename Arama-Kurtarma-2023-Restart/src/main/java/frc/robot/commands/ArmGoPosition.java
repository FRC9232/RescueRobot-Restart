// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
;

public class ArmGoPosition extends CommandBase {
  private final ArmSubsystem m_armSystem;
  private final double m_position;
  private double m_speed;
  private double m_stabilizationSpeed;
  private double m_direction;
  private double kF;
  private double kc = 0.1;
  private double generalError ;
  private boolean m_positionType; //1-direct position, 0-difference with the current pos
  private double m_targetPosition;
  private double m_lastError;

  /** Creates a new MainArmGoPosition. */
  public ArmGoPosition(ArmSubsystem armSystem, double position, boolean positionType) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSystem = armSystem;
    m_position = position;
    m_positionType = positionType;
    addRequirements(m_armSystem);
    m_stabilizationSpeed = 0.1;
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentPosition = m_armSystem.frontArm.getSelectedSensorPosition();
    
    if(!m_positionType){
      m_targetPosition = currentPosition + m_position;
    }else{
      m_targetPosition = m_position;
    }

    double error = m_targetPosition - currentPosition;
    double maxSpeed = 0.4;
    kF = maxSpeed / error;
    if(error <0){
      kF = kF * -1;
    
    }else{
      maxSpeed = 0.8;
      kF = maxSpeed / error;
    }
    m_armSystem.setPercent("front", error*kF);
   
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    generalError = m_targetPosition - m_armSystem.frontArm.getSelectedSensorPosition();
    System.out.println(generalError);
    
    
    m_armSystem.setPercent("front", generalError*kF);
    if(generalError <= 50 && generalError >=-50){
      m_armSystem.setPercent("front", generalError*kF+kc);

      //try this; if the error same with previous one then cancel the command, 
      //otherwise stragle to check if arm can stay the position
      
      if(m_lastError == generalError){
        this.cancel();
      }
      
  
    }
    else{
      //if the power of the motor is not enough to move the arm, increase the power
      if(m_lastError == generalError){  
        m_armSystem.setPercent("front", generalError*kF+kc);
      }else{
        m_armSystem.setPercent("front", generalError*kF);
      }
    }
    m_lastError = generalError;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   

    if(generalError <= 50 && generalError >=-50){


      m_armSystem.setPercent("front", generalError*kF+kc);
      return true;
    }
  return false;
}
}
