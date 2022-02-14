package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team254.frc2020.Constants;
import com.team254.frc2020.RobotState;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.frc2020.planners.DriveMotionPlanner;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonFXChecker;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveOutput;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

private class drive {
    private final TalonFX mLeftMaster1, mRightMaster1, mLeftSlave1, mRightSlave1;

    private void configureTalon(TalonFX talon, boolean left, boolean main_encoder_talon) {
        // general
        talon.setInverted(!left);
        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);
    }

    private drive(){
        mLeftMaster1 = new TalonFX(1);
        mRightMaster1 = new TalonFX(2);
        mLeftSlave1.set(ControlMode.Follower, 1);
        mRightSlave1.set(ControlMode.Follower, 2);

        configureTalon(mLeftMaster1, true, false);
        configureTalon(mLeftSlave1, true, false);
        configureTalon(mRightSlave1, false, false);
        configureTalon(mRightMaster1, false, true);
    }

    //if turn is >0 increase right speed to turn left, if turn < 0 increase left speed to turn right
    public void setOpenLoop(double throttle, double turn) {
        if(turn > 0){
            throttle = turn + throttle;
            mRightMaster1.set(ControlMode.PercentOutput, throttle);
        }
        else if (turn < 0){
            throttle = throttle - turn;
            mLeftMaster1.set(ControlMode.PercentOutput, throttle);
        }
    }

    public void stop(){
        mLeftMaster1.set(ControlMode.PercentOutput, 0.0);
        mRightMaster1.set(ControlMode.PercentOutput, 0.0);
    }

    
}
