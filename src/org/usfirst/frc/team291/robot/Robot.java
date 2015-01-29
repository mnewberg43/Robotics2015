package org.usfirst.frc.team291.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.RGBImage;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;
//import YellowToteTracker;


import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

/**
 * 
 * @author Gabriel Dougherty, Supreme Duncan Paterson, Aiden Brzuz, Matt and Matt
 * 
 * Cartesian Drive Guide:
 *                                Y axis, rotation, X axis,   gyro Angle
 * myRobot.mecanumDrive_Cartesian(x,      y,        rotation, gyroAngle);
 * 
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	int autoLoopCounter;
	int session;
	Image frame;
	AxisCamera camera;
	Timer myTimer = new Timer();
	Gyro gyro;
	double Kp = 0.03;
	double angle;
	Compressor compressor = new Compressor(0);
	DoubleSolenoid solenoid = new DoubleSolenoid(0,1);
	DigitalOutput blinky = new DigitalOutput(0);
	double stickX, stickY;
	double correctiveAngle ,
			currentAngle ;
	double deadSpotMin = -0.2;
	double deadSpotMax = 0.2;
	double autonomousTurn = 1;
	double angleScaling = -60 ;
	boolean firstRun ;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
	    myRobot = new RobotDrive(0,1,2,3);
	    gyro = new Gyro(0);
	    frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
		

        // open the camera at the IP address assigned. This is the IP address that the camera
        // can be accessed through the web interface.
	    camera = new AxisCamera("10.2.91.11");
        compressor.setClosedLoopControl(true);
        stick = new Joystick(0);
        blinky.set(false);
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	myTimer.reset();
    	myTimer.start();
    	gyro.reset();
    	correctiveAngle = 0;
    	SmartDashboard.putNumber("Corrective", correctiveAngle);
    	SmartDashboard.putNumber("Gyro Angle", currentAngle);
    	SmartDashboard.putNumber("Angle", (currentAngle-correctiveAngle)/angleScaling);
    	firstRun = true ;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
//    	correctiveAngle = gyro.getAngle()/180;
    	if (firstRun)
    	{
    	Timer.delay(0.005);
    	correctiveAngle = gyro.getAngle();
    	firstRun = false;
    	}
    	LiveWindow.run();
    	while (myTimer.get() <= autonomousTurn) {
    		currentAngle = gyro.getAngle();
    		myRobot.mecanumDrive_Cartesian(0.3, (currentAngle-correctiveAngle)/angleScaling, 0, 0);
    	}
    	LiveWindow.run();
//    	gyro.reset();
    	while (myTimer.get() > (autonomousTurn * 2) && myTimer.get() <= (3 * autonomousTurn)) {
    		myRobot.mecanumDrive_Cartesian(0.0, (currentAngle-correctiveAngle)/angleScaling, 0.6,  0);
    	}
    	LiveWindow.run();
//    	gyro.reset();
    	while (myTimer.get() > (4 * autonomousTurn) && myTimer.get() <= (5 * autonomousTurn)) {
    		currentAngle = gyro.getAngle();
    		myRobot.mecanumDrive_Cartesian(-0.3, (currentAngle-correctiveAngle)/angleScaling, 0, 0);
    	}
    	LiveWindow.run();
//    	gyro.reset();
    	while (myTimer.get() > (6 * autonomousTurn) && myTimer.get() <= (7 * autonomousTurn)) {
    		currentAngle = gyro.getAngle();
    		myRobot.mecanumDrive_Cartesian(0, (currentAngle-correctiveAngle)/angleScaling, -0.6, 0);
    	}
    	while (myTimer.get() > (8 * autonomousTurn) && myTimer.get() <= (9 * autonomousTurn)) {
    		currentAngle = gyro.getAngle();
    		myRobot.mecanumDrive_Cartesian(0, (currentAngle-correctiveAngle)/angleScaling, 0, 0);
    	}
//    	gyro.reset();
    	// forward (the beginnings of a bad set of nested if statements
//    	if (!myTimer.hasPeriodPassed(autonomousTurn)) {
//    		myRobot.mecanumDrive_Cartesian(0.3, correctiveAngle, 0, 0);
//    	}
//    	else {
//    		if (myTimer.hasPeriodPassed(2 * autonomousTurn && )){
//    			myRobot.mecanumDrive_Cartesian(0, correctiveAngle, 0.3, 0);
//    		}
//    	}
//    	

    	LiveWindow.run();
	}
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
//    	NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
//
//        while (isOperatorControl() && isEnabled()) {
//            camera.getImage(frame);
//            NIVision.imaqDrawShapeOnImage(frame, frame, rect,
//                    DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
//
//            CameraServer.getInstance().setImage(frame);
    	gyro.reset();
    	myTimer.stop();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
        while (isOperatorControl() && isEnabled()) {
	        	angle = gyro.getAngle();
        	if (stick.getY() < deadSpotMax && stick.getY() > deadSpotMin){
        		stickY = 0;
        	}
        	else{
        		stickY = -stick.getY();
        	}
        	if (stick.getX() < deadSpotMax && stick.getX() > deadSpotMin){
        		stickX = 0;
        	}
        	else{
        		stickX = -stick.getX();
        	}
	    	myRobot.mecanumDrive_Cartesian(-stick.getY(), stick.getRawAxis(4), stick.getX(), -angle*Kp);
	        camera.getImage(frame);
	        NIVision.imaqDrawShapeOnImage(frame, frame, rect,
	                DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
	
	        CameraServer.getInstance().setImage(frame);
	        if (stick.getRawButton(1)){
	        	solenoid.set(DoubleSolenoid.Value.kForward);
	        }
	        if (stick.getRawButton(2)){
	        	solenoid.set(DoubleSolenoid.Value.kReverse);
	        }
	        if (stick.getRawButton(3)){
	        	blinky.set(true);
	        }
	        if (stick.getRawButton(4)){
	        	blinky.set(false);
	        }
	        LiveWindow.run();
	        
	        Timer.delay(0.005);
	    }
}
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    LiveWindow.run();
    }
    
}