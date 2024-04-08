// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class DefaultArmDrive extends CommandBase {
  private final ArmSubsystem m_Arm;
  private final double m_forward;
  private final String m_armType;
  //private final DoubleSupplier m_rotation;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  //public DefaultDrive(DriveTrain subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
  public DefaultArmDrive(ArmSubsystem subsystem, double forward, String armType) {
    m_Arm = subsystem;
    m_forward = forward;
    m_armType = armType;
    addRequirements(m_Arm);
  }

  @Override
  public void execute() {
    double power = m_forward;
        //double turn = m_rotation.getAsDouble();
    m_Arm.teleopArcadeDrivePercent(power, m_armType);
  }

  public double deadband(double value){
    if(value < 0.1 && value > -0.1){
      return 0;
    }
    else{
      return value;
      //return (value - (Math.abs(value)/value*0.1))/(1-0.1);
    }
  }
}
