package org.usfirst.frc.team291.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.RGBImage;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import com.ni.vision.NIVision.DrawMode;

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
import edu.wpi.first.wpilibj.BuiltInAccelerometer;

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
	Joystick stick1, stick2;
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
	static double deadSpotMin = -0.2;
	static double deadSpotMax = 0.2;
	double autonomousTurn = 1;
	double angleScaling = -60 ;
	boolean firstRun ;
	CANTalon motor1, motor2, motor3, motor4;
	String currentDriveMode = "normal"; // TODO: IMO we should change this, I dunno, maybe
	String driveModes[] = {"normal","fieldCentric"};
	double rotation, commandX, commandY, fieldCentric, commandArm;
	Talon armLeft;
	Talon armRight;
	DoubleSolenoid armPneumatics;
	BuiltInAccelerometer accl;
	double acclX, acclY, acclZ;
	double a1, a2, a3, aCurrent;
	double v1, v2, v3, vCurrent;
	double dCurrent;
	double timeInterval;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	motor1 = new CANTalon(0);
    	motor2 = new CANTalon(1);
    	motor3 = new CANTalon(2);
    	motor4 = new CANTalon(3);
	    myRobot = new RobotDrive(motor1,motor2,motor3,motor4);
	    armLeft = new Talon(1);
	    armRight = new Talon(0);
	    gyro = new Gyro(0);
	    frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
	    
		

        // open the camera at the IP address assigned. This is the IP address that the camera
        // can be accessed through the web interface.
	    camera = new AxisCamera("10.2.91.11");
        compressor.setClosedLoopControl(true);
        stick1 = new Joystick(0);
        blinky.set(false);
        commandArm = stick2.getY() ;
        accl = new BuiltInAccelerometer();
        a2 = 99;
        a1 = 99;
        a3 = 99;
        aCurrent = 99;
        v2 = 99;
        v1 = 99;
        v3 = 99;
        vCurrent = 99;
        dCurrent = 99;
        timeInterval = 0.5; // TODO: Use real interval
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
    	firstRun = true;
    }
    
    private double integralOf(double lower, double middle, double upper, double current) {
    	double d = 0;
    	if (lower == 99) {
    		lower = current;
    	}
    	else if (middle == 99) {
    		middle = current;
    	}
    	else if (upper == 99) {
    		upper = current;
    	}
    	if (upper != 99) {
        	lower = middle;
        	middle = upper;
        	upper = current;
        }
    	d = (1/3) * timeInterval * (lower + 4 * middle + upper); // Simpson's rule
    	return d;
    }
    	
    public void grabTote() {
    	armLeft.set(commandArm*-1.05) ;
		armRight.set(commandArm*-1) ;
    	armPneumatics.set(DoubleSolenoid.Value.kForward);
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
    	myTimer.reset();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
        while (isOperatorControl() && isEnabled()) {
        	commandY = -stick1.getY();
        	commandX = stick1.getX();
        	rotation = stick1.getRawAxis(4);
	        angle = gyro.getAngle();
        	if (stick1.getY() < deadSpotMax && stick1.getY() > deadSpotMin){
        		stickY = 0;
        	}
        	else{
        		stickY = -stick1.getY();
        	}
        	if (stick1.getX() < deadSpotMax && stick1.getX() > deadSpotMin){
        		stickX = 0;
        	}
        	else{
        		stickX = -stick1.getX();
        	}
        	
	        camera.getImage(frame);
	        NIVision.imaqDrawShapeOnImage(frame, frame, rect,
	                DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
	        CameraServer.getInstance().setImage(frame);
	        
	        // Control Buttons and Such
	        if (stick1.getRawButton(1)) {
	        	solenoid.set(DoubleSolenoid.Value.kForward);
	        }
	        else if (stick1.getRawButton(2)) {
	        	solenoid.set(DoubleSolenoid.Value.kReverse);
	        }
	        else if (stick1.getRawButton(3)) {
	        	blinky.set(true);
	        }
	        else if (stick1.getRawButton(4)) {
	        	blinky.set(false);
	        }
	        else if (stick1.getRawButton(5)) { // TODO: Assign to correct button
	        	currentDriveMode = driveModes[0];
	        }
	        else if (stick1.getRawButton(6)) { // TODO: Assign to correct button
	        	currentDriveMode = driveModes[1];
	        }
	        
	        // Drive Mode Toggles
	        if (currentDriveMode.equals(driveModes[0])) { // Normal
        		fieldCentric = 0;
        	}
	        else if (currentDriveMode.equals(driveModes[1])){ // Field-Centric
	        	fieldCentric = gyro.getAngle();
	        }
	        if ( stick2.getRawButton(1) )
    		{
    			armPneumatics.set(DoubleSolenoid.Value.kForward);
    		}
    		else if (stick2.getRawButton(2))
    		{
    			armPneumatics.set(DoubleSolenoid.Value.kReverse);
    		}
    		else 
    		{
    			armPneumatics.set(DoubleSolenoid.Value.kOff);
    		}
	        commandArm = stick2.getY();
    		
    		if ( (commandArm < deadSpotMax) && (commandArm > deadSpotMin) )
    		{
    			commandArm = 0 ;

    		}
    		commandArm = commandArm * .7 ;
    		
    		if (commandArm == 0 )
    		{
    			if (stick2.getRawButton(5))
        		{
    				armLeft.set(.2);
        		}
        		else if (stick2.getRawButton(6))
        		{
        			armRight.set(.2);
        		}
        		else
        		{
        			armLeft.set(0);
        			armRight.set(0); 
        		}
    		}
    		else
    		{
    			armLeft.set(commandArm*-1.05);
    			armRight.set(commandArm*-1);
    		}
	        myRobot.mecanumDrive_Cartesian(commandY, rotation, commandX, fieldCentric);
	        
	        
	        
	        
	        acclX = accl.getX();
	        acclY = accl.getY();
	        acclZ = accl.getZ(); // TODO: Magic Vectors
	        
	        aCurrent = 1; //acceleration read from sensor and used with vector stuff
	        vCurrent += integralOf(a1, a2, a3, aCurrent);
	        dCurrent += integralOf(v1, v2, v3, vCurrent);
	        
	        SmartDashboard.putNumber("dCurrent", dCurrent);
	        
	        
/*
 * 	1/3h(f_0+4f_1+f_2)
 */
	        
	        SmartDashboard.putNumber("acclX", acclX);
	        SmartDashboard.putNumber("acclY", acclY);
	        SmartDashboard.putNumber("acclZ", acclZ);
	        
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