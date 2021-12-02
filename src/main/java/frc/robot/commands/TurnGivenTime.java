/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

// import edu.wpi.first.wpilibj.interfaces.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

public class TurnGivenTime extends CommandBase {

    //insantiate global variables
	double turnPercent, forwardPercent;
    Timer timer;
    double sp;
    double tm;
    
 
	public TurnGivenTime(double speed, double time) {
        
        //initialize variables
        sp = speed;
        tm = time;
        
        //set command to be interruptible
		//setInterruptible(true);

        timer = new Timer();
    }
    

	// Called just before this Command runs the first time
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
        Robot.m_frontLeft.set (VictorSPXControlMode.PercentOutput, sp);
    
        Robot.m_frontRight.set (VictorSPXControlMode.PercentOutput, -sp);
       
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
        return timer.get() >= tm; 
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
