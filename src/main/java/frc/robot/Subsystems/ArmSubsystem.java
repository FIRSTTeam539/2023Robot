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
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax arm = new CANSparkMax(ArmConstants.kArmSparkMaxCANID, MotorType.kBrushless);
  private final Spark armShoot = new Spark(ArmConstants.kShootMotorPort);

  /** Create a new ArmSubsystem. */
    public ArmSubsystem() {
        // Start arm at rest in neutral position
        arm.setIdleMode(IdleMode.kBrake);
        // Start arm at rest in neutral position

        //Unsure if code below is correct
          //setGoal(ArmConstants.kArmOffsetRads);
    }
    private void shootArm(Double armShootControl){
      // - shoot cargo; + intake cargo
        armShoot.set(armShootControl);
        ///armShoot.set(armShootControl * ArmConstants.kshootRate - armIntakeControl * ArmConstants.kintakeRate);
    }
    /**
     *moves arm and shoots arm
     * 
     * @param armMoveControl control to move the arm with, + is up, - is down
     * @param shootArmControl control to shoot with, - is shoot, + is intake
     * 
     */
    public void moveArm(double armMoveControl, double shootArmControl){

        double shootArmAxis = -MathUtil.clamp(armMoveControl, ArmConstants.kMin, ArmConstants.kMax); // Apply axis clamp and invert for driver control

        if (Math.abs(shootArmAxis) > ArmConstants.kDeadzone) arm.set(shootArmAxis * ArmConstants.karmRate);
        else arm.set(0);

      shootArm(shootArmControl);
        
        
        SmartDashboard.putNumber("arm power", shootArmAxis * ArmConstants.karmRate); // put arm speed on Smartdash

        SmartDashboard.putNumber("arm enc value", arm.getEncoder().getPosition()); // put encoder value on SmartDash
    }


  public double getMeasurement() {
    return arm.getEncoder().getPosition();
  }
}