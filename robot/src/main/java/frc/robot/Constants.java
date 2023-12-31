// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class IntakeConstants 
  {
    public static final double CollectMotorSpeed = 0.5;
    public static final int KSolenoidID = 1;
    public static final int KMotorID = 20;
  } 

  public static class Arm_constants{
    public static final int KMotorID = 20;
    public static final double CollectMotorSpeed = 0.5;

    public static final double mmVelocity = 5.0;
    public static final double mmAcceleration = 10.0;
    public static final double mmJerk = 50;

    public static final double KP = 24.0;
    public static final double KD = 0.1;
    public static final double KV = 0.12;
    public static final double KS = 0.25;

    public static final double minimumMistake = 30;
    public static final double minimumTime = 10;

    public static final double PeakForwardVoltage = 11.5;
    public static final double PeakReverseVoltage = -11.5;
    public static final double SensorToMechanismRatio = 50;
  }
    

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
