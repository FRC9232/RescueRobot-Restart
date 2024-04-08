// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;

public class AutoClimbGetReady extends CommandBase {
  private final DriveTrain m_DriveTrain;
  private final ArmSubsystem m_Arm;
  private final double m_frontSpeed;
  private final double m_backSpeed;

  /** Creates a new AutoClimb. */
  public AutoClimbGetReady(DriveTrain driveTrain, ArmSubsystem armSubsystem, double frontSpeed, double backSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
   m_DriveTrain = driveTrain;
   m_Arm = armSubsystem;
   m_frontSpeed = frontSpeed;
   m_backSpeed = backSpeed;
   addRequirements(m_DriveTrain); 
   addRequirements(m_Arm); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_DriveTrain.driveIt();
    m_Arm.setPercent("front", 0.4);
    m_Arm.setPercent("back", 0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
    if(m_Arm.limitSwitchBackArm.get()){
      m_Arm.setPercent("back", 0.1);
    }

    if(m_Arm.limitSwitchFrontArm.get()){
      m_Arm.setPercent("front", 0.05);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Arm.limitSwitchBackArm.get()){
      m_Arm.setPercent("back", 0.1);
    }

    if(m_Arm.limitSwitchFrontArm.get()){
      m_Arm.setPercent("front", 0.05);
    }

    if(m_Arm.limitSwitchBackArm.get() && m_Arm.limitSwitchFrontArm.get()){
      return true;
    }
    return false;
  }
}
