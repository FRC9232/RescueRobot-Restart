// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
;

public class deneme extends CommandBase {
  private final ArmSubsystem m_armSystem;
  private final double m_position;
  private double m_speed;
  private double m_stabilizationSpeed;
  private double m_direction;
  private double kF;
  private final double kc = 0.05;

  /** Creates a new MainArmGoPosition. */
  public deneme(ArmSubsystem armSystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSystem = armSystem;
    m_position = position;
    addRequirements(m_armSystem);
    m_stabilizationSpeed = 0.1;
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    double currentPosition = m_armSystem.frontArm.getSelectedSensorPosition();
    double error = m_position - currentPosition;
    double maxSpeed = 0.2;
    kF = -1 * maxSpeed / error;
   
   
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = m_position - m_armSystem.frontArm.getSelectedSensorPosition();
    m_armSystem.setPosition(error*kF+kc);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_armSystem.frontArm.getSelectedSensorPosition() == m_position){

      return true;
    }
     
    
  return false;
}
}
