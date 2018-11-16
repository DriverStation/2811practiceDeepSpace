/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import edu.wpi.first.wpilibj.SPI;

/**
 * An example command. You can replace me with your own command.
 */
public class PathfinderWaypoints extends Command {

	EncoderFollower left;
    EncoderFollower right;

	private final AHRS gyro = new AHRS(SPI.Port.kMXP); // remember that the number is the port number

	public PathfinderWaypoints() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drive);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		//gyro.setSensitivity(0); //k Volts / Degree / Second
        gyro.reset();
        Robot.drive.leadL.setSelectedSensorPosition(0, 0, 10000);
        Robot.drive.leadR.setSelectedSensorPosition(0, 0, 10000);

        System.out.println("Ye start'n heeere!!");

		Waypoint[] points = new Waypoint[] {
			// Waypoint(x, y, degrees/radains) //Pathfinder.d2r(90)
			//new Waypoint(12, 120, Pathfinder.d2r(0)), 
			//new Waypoint(84, 120, Pathfinder.d2r(0)),
			//new Waypoint(120, 156, Pathfinder.d2r(90)),
			//new Waypoint(120, 216, Pathfinder.d2r(90)),
			//new Waypoint(156, 252, Pathfinder.d2r(0)),
            //new Waypoint(216, 252, Pathfinder.d2r(0))
            new Waypoint(0, 0, Pathfinder.d2r(0)),
            //new Waypoint(20*12, 20*12, Pathfinder.d2r(-90)),
            //new Waypoint(40*12, 40*12, Pathfinder.d2r(0))
            new Waypoint(40*12, 0, Pathfinder.d2r(0))
        };

		Trajectory.Config config = new Trajectory.Config(
            Trajectory.FitMethod.HERMITE_CUBIC, // either cubic or quintic
			Trajectory.Config.SAMPLES_LOW, // HIGH, LOW, or FAST
			0.03, // Duration (seconds)
			144, // Max Velocity (m/sec)
			60, // Max Acceleration (m/s/s)
			720.000389 // Max Jerk (m/s/s/s)
		);

		Trajectory trajectory = Pathfinder.generate(points, config);

		// in meters
		double wheelbase_width = 18;

		// generate left and right using trajectory as center
        TankModifier modifier = new TankModifier(trajectory)
                                    .modify(wheelbase_width);

        //Robot.drive.leadL.setInverted(true);
        //Robot.drive.leadL.setSensorPhase(false);

		left = new EncoderFollower(modifier.getLeftTrajectory()); // get the left
		right = new EncoderFollower(modifier.getRightTrajectory()); // get the right

		// encoder_position is current cumulative, getEncPosition()
		// 256 is ticks per revolutions
		// wheel diameter is diameter of wheels (or pully/track system) in meters
		// left.configureEncoder(encoder_position, 256, wheel diameter);
		left.configureEncoder(Robot.drive.leadL.getSelectedSensorPosition(0), 128, 6);
		right.configureEncoder(Robot.drive.leadR.getSelectedSensorPosition(0), 256, 6);

		// left.configurePIDVA(P, I, D, velocity ratio, acceleration)
		left.configurePIDVA(0.0, 0.0, 0.0, 1 / 144.0, 0); // 0.8
        right.configurePIDVA(0.0, 0.0, 0.0, 1 / 144.0, 0);
        
        System.out.println("'nd 'ere 'e 'nd");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double outputL = left.calculate(-Robot.drive.leadL.getSelectedSensorPosition(0));
        double outputR = right.calculate(Robot.drive.leadR.getSelectedSensorPosition(0));

        double headingA = gyro.getAngle();
        System.out.println("Gyro: " + headingA);
        double headingT = Pathfinder.boundHalfDegrees(left.getHeading());
        System.out.println("Target: " + headingT);

        double angleDiff = Pathfinder.boundHalfDegrees(headingT - headingA);
        
        angleDiff=0; // GET RID OF THIS WHN ON GROUND
        System.out.println("angleDiff: " + angleDiff);
        double turn = 0.8 * (-1.0 / 80.0) * angleDiff;
        System.out.println("turn: " + turn);

        System.out.println("Output L: " + outputL);
        System.out.println("Output R: " + outputR);

        //if (outputL > 1) outputL = 1;
        //if (outputR > 1) outputR = 1;
        SmartDashboard.putNumber("Output L", outputL);
        SmartDashboard.putNumber("Output R", outputR);

        System.out.println("Sensor L (neg): " + -Robot.drive.leadL.getSelectedSensorPosition(0)*2);
        System.out.println("Sensor R: " + Robot.drive.leadR.getSelectedSensorPosition(0));


		Robot.drive.leadL.set(ControlMode.PercentOutput, (-(outputL - turn)) );
		Robot.drive.leadR.set(ControlMode.PercentOutput, (outputR + turn) );
	}

	public void initDefaultCommand() { 
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when a+nother command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.drive.leadL.set(ControlMode.PercentOutput, 0);
		Robot.drive.leadR.set(ControlMode.PercentOutput, 0);
	}
}
