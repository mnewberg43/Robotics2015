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

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	int autoLoopCounter;
	int session;
	Image frame;
	AxisCamera camera;
	Compressor compressor = new Compressor(0);
	DoubleSolenoid solenoid = new DoubleSolenoid(0,1);
	Timer myTimer = new Timer();
	Gyro myGyro;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    myRobot = new RobotDrive(0,1,2,3);
    myGyro = new Gyro(0);
    frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // open the camera at the IP address assigned. This is the IP address that the camera
        // can be accessed through the web interface.
//        camera = new AxisCamera("10.2.91.11"); TODO
        compressor.setClosedLoopControl(true);
        
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	myTimer.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        double angle = myGyro.getAngle();
    	while (isAutonomous() == true) {
    		myRobot.mecanumDrive_Cartesian(0, -angle/30, 0, 0);
    	}
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
//
//    
//            Time
//
//    	//YellowToteTracker tracker; TODO: get this to work

        stick = new Joystick(0);
        SmartDashboard.putString("haha","haha");
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    while (isOperatorControl() && isEnabled()) {
    	myRobot.mecanumDrive_Cartesian(-stick.getY(), stick.getZ(), stick.getX(), 0);
        Timer.delay(0.005);
        
        NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);

    
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