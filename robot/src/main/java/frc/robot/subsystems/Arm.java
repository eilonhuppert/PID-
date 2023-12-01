// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  
  private TalonFX motor;

  private Arm(){
    this.motor = new TalonFX(Constants.Arm_constants.KMotorID);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    MotionMagicConfigs mm = new MotionMagicConfigs();


    mm.MotionMagicCruiseVelocity = Constants.Arm_constants.mmVelocity;
    mm.MotionMagicAcceleration = Constants.Arm_constants.mmAcceleration;
    mm.MotionMagicJerk = Constants.Arm_constants.mmJerk;
    configs.MotionMagic = mm;

    configs.Slot0.kP =  Constants.Arm_constants.KP;
    configs.Slot0.kD =  Constants.Arm_constants.KD;
    configs.Slot0.kV =  Constants.Arm_constants.KV;
    configs.Slot0.kS =  Constants.Arm_constants.KS;

    configs.Voltage.PeakForwardVoltage = Constants.Arm_constants.PeakForwardVoltage;
    configs.Voltage.PeakReverseVoltage = Constants.Arm_constants.PeakReverseVoltage;
    configs.Feedback.SensorToMechanismRatio = Constants.Arm_constants.SensorToMechanismRatio;

    // gives code to TalonFX
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++){
      status = motor.getConfigurator().apply(configs);
      if (status.isOK()){
        break;
      }
    }
    if (!status.isOK()){
      System.out.println("Could not apply configs, erro code: " + status.toString());
    }
    
    //make sure we start at 0.
    motor.setRotorPosition(0);
  }

  public static Arm instance;
  
  // singelton
  public static Arm getInstance(){
    if (instance == null){
      instance = new Arm();
    }
    return instance;
  }

  public void putArmInPos(double place){
    motor.setControl(motionMagic.withPosition(place));
  }
  public double getPosArm(){
    return this.motor.getPosition().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
