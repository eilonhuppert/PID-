// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intakesubsystem extends SubsystemBase {
  /** Creates a new Intakesubsystem. */
  
  private Solenoid solenoid;
  private TalonFX motor;
  private Intakesubsystem() {
    this.solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.KMotorID);
    this.motor = new TalonFX(Constants.IntakeConstants.KMotorID);
  }
  public static Intakesubsystem instance;
  
  public static Intakesubsystem getInstance(){
    if (instance == null){
      instance = new Intakesubsystem();
    }
    return instance;
  }

  public void setSolenoid(boolean value) {
    this.solenoid.set(value);
  }
  public void setMotor(double speed){
    this.motor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
