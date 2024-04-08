// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class ArmSubsystem extends SubsystemBase {
  
//public CANSparkMax frontArm = new CANSparkMax(6, MotorType.kBrushed);
//public CANSparkMax backArm = new CANSparkMax(7, MotorType.kBrushed);
public WPI_TalonSRX frontArm = new WPI_TalonSRX(3);
public WPI_TalonSRX backArm = new WPI_TalonSRX(4);

public DigitalInput limitSwitchFrontArm = new DigitalInput(9);
public DigitalInput limitSwitchBackArm = new DigitalInput(0);

public boolean frontArmisOnTarget = false;
public boolean backArmisOnTarget = false;

public double kF;
public double kP;

public double manualSpeed = 0.0;

/** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
   
    //backArmEncoder.setInverted(true);

   
    frontArm.setInverted(false);
    backArm.setInverted(true);
    
    
    resetEncoders();
  }

  public void setPercent(String armType, double speed){

    if(armType == "front"){
    frontArm.set(ControlMode.PercentOutput, speed);
    }
    else if(armType == "back"){
      backArm.set(ControlMode.PercentOutput, speed);
    }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Back Arm Position", backArm.getSelectedSensorPosition());
    SmartDashboard.putNumber("Front Arm Position", frontArm.getSelectedSensorPosition());

    SmartDashboard.putBoolean("Front Arm Limit Switch", limitSwitchFrontArm.get());
    SmartDashboard.putBoolean("Back Arm Limit Switch", limitSwitchBackArm.get());
    if(limitSwitchFrontArm.get()){
      frontArm.setSelectedSensorPosition(0);
    }
    if(limitSwitchBackArm.get()){
      backArm.setSelectedSensorPosition(0);
    }

    kF = SmartDashboard.getNumber("Arm kF", 1);
    kP = SmartDashboard.getNumber("Arm kP", 1);

    
   
   
  }

  public void resetEncoders() {
    frontArm.setSelectedSensorPosition(0);
    backArm.setSelectedSensorPosition(0);
  }


  public void setPosition(double position){
    frontArm.set(ControlMode.Position, position);
  }

  public void setPosition2(double position){
      double currentPosition = frontArm.getSelectedSensorPosition();
      double error = position - currentPosition;
      double maxSpeed = 0.2;
      double kF = -1 * maxSpeed / error;
      double kc = 0.05;
      System.out.println("error:" + error);
      double oldSpeed = 0;
      while(frontArm.getSelectedSensorPosition() != position){
        error = position - frontArm.getSelectedSensorPosition();
        if(oldSpeed != error*kF+kc){
            oldSpeed = error*kF+kc;


            System.out.println(oldSpeed);
        }
       
        frontArm.set(ControlMode.PercentOutput, error*kF+kc);
      }
    }
   


  public void setPositionBackArm(double position){
    backArm.set(ControlMode.Position, position);
  }

  public void teleopArcadeDrivePercent(double power, String armType){

    if(armType == "front"){

      frontArm.set(ControlMode.PercentOutput, power);
    }
    else if(armType == "back"){
      backArm.set(ControlMode.PercentOutput, power);
    }
    //backArm.set(ControlMode.PercentOutput, turn);
  }
}
