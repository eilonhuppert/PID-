// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;


public class Arm_command extends CommandBase {

  private final Arm arm = Arm.getInstance();
  private int timer = 0;
  private double place;

  private double setpoint;
  /** Creates a new Arm_command. */
  
  public Arm_command(double place) {
    this.place = place;
    this.addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.putArmInPos(place);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    if (Math.abs(arm.getPosArm() - this.setpoint) <= Constants.Arm_constants.minimumMistake) {
      if (timer == Constants.Arm_constants.minimumTime){
        timer = 0;
        return true;
    
      }else{
        timer++;
        return false;
      }

    }else{
      timer = 0;
    return false;
    }
  }
}
