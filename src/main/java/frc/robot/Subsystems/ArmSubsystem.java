package frc.robot.Subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax armMotor1 = new CANSparkMax(ArmConstants.kArmSparkMaxCANID1, MotorType.kBrushless);
  private final CANSparkMax armMotor2 = new CANSparkMax(ArmConstants.kArmSparkMaxCANID2, MotorType.kBrushless);
  /** Create a new ArmSubsystem. */
    public ArmSubsystem() {
        // Start arm at rest in neutral position
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);
        // Start arm at rest in neutral position

        //Unsure if code below is correct
          //setGoal(ArmConstants.kArmOffsetRads);
    }
    /**
     *sets arm to a possition
     * 
     * @param armMoveControl control to move the arm with, + is up, - is down
     *
     * 
     */
    public void setArm(double armMoveControl){

        double shootArmAxis = -MathUtil.clamp(armMoveControl, ArmConstants.kMin, ArmConstants.kMax); // Apply axis clamp and invert for driver control
        armMotor1.set(shootArmAxis * -ArmConstants.karmRate);
        armMotor2.set(shootArmAxis * ArmConstants.karmRate);
        
        SmartDashboard.putNumber("arm power", shootArmAxis * ArmConstants.karmRate); // put arm speed on Smartdash
        SmartDashboard.putNumber("arm1 enc value", armMotor2.getEncoder().getPosition()); 
        SmartDashboard.putNumber("arm2 enc value", armMotor2.getEncoder().getPosition()); // put encoder value on SmartDash
    }
    /**
     * lowers arm
     * @ returns command to lower arm
     * 
     */
    public Command lowerArm (){
     return this.run(() -> this.setArm(-ArmConstants.ARM_OUTPUT_POWER));

    }
    /**
     * raises arm
     * @return returns command to raise arm
     */
    public Command raiseArm (){
      return this.startEnd(() -> this.setArm(ArmConstants.ARM_OUTPUT_POWER), () -> this.setArm(0));
 
    }

    public Command disableArm () {
      return this.startEnd(() -> this.setArm(0), () -> this.setArm(0));
    } 

}