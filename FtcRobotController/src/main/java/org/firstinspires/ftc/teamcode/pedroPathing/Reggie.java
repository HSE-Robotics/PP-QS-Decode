package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.EnumMap;
import java.util.Objects;


public class Reggie {

    // Declare hardware components as public to be accessible in OpModes
    private Timer pathTimer;
    public static Pose poseFromAuto = new Pose(0,0,0);
    public static Pose scoringPose = new Pose(0,0,0);
    public static Pose startingPose;
    public DcMotor intakeMotor = null;
    public DcMotorEx leftShooterMotor = null;
    public DcMotorEx rightShooterMotor = null;

    public DcMotorEx parkingLift = null;
    public Servo sorterServo = null;
    public Servo rightStopper = null;
    public Servo leftStopper = null;
    public CRServo leftIndexerServo = null;
    public CRServo rightIndexerServo = null;
    public ElapsedTime idlerTime;
//    public ColorSensor rightColorSensor = null;
//    public ColorSensor leftColorSensor = null;
    public boolean usingPIDF;

    public enum StoppersStates{
        STOP,
        PASS
    }
    StoppersStates stoppersStates = StoppersStates.STOP;
    public double TARGET_VELOCITY_LEFT = 1100;
    public double TARGET_MIN_VELOCITY_LEFT = 900;
    public double TARGET_VELOCITY_RIGHT = 1300;
    public double TARGET_MIN_VELOCITY_RIGHT = 1200;

    public double TARGET_VELOCITY_FIRST = 1150;
    public double TARGET_MIN_VELOCITY_FIRST = 950;

    public enum SIDES{
        RIGHT,
        LEFT,
        FIRST
    }

    public SIDES shootingSide = SIDES.RIGHT;
    public double sortPositionMiddle = 0.45;
    public double sortPositionRight = 0.05;
    public double sortPositionLeft = 0.9;

    // FIND THIS VALUES
    public double leftStopperSTOP = 0.87;
    public double leftStopperPASS = 0.6;
    public double rightStopperPASS = 0.55;
    public double rightStopperSTOP = 0.27;
    public double P = 38;
    public double F = 14;

    // HardwareMap object
    private HardwareMap hwMap = null;


    /* Constructor */
    public Reggie() {

    }

    /* Initialize hardware */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        //poseFromAuto = new Pose();

        intakeMotor = hwMap.get(DcMotor.class, "int");
        leftShooterMotor = hwMap.get(DcMotorEx.class, "LS");
        rightShooterMotor = hwMap.get(DcMotorEx.class, "RS");

        // Define and Initialize Servos
        rightStopper = hwMap.get(Servo.class, "RStopper");
        leftStopper = hwMap.get(Servo.class, "LStopper");
        leftIndexerServo = hwMap.get(CRServo.class, "LServo");
        rightIndexerServo = hwMap.get(CRServo.class, "RServo");
        sorterServo = hwMap.get(Servo.class,"sorter");



        //Define and Initialize Sensors
//        rightColorSensor = hwMap.get(ColorSensor.class, "colorSensorRight");
//        leftColorSensor = hwMap.get(ColorSensor.class, "colorSensorLeft");


        idlerTime = new ElapsedTime();

        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIndexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set all motors to zero power
        intakeMotor.setPower(0);
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);



        //leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pathTimer = new Timer();

        // Set all motors to run without encoders
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);

        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        //rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        //Set all Sensors

        usingPIDF = true;

        // Set servo initial position
        this.setSorterServoPosition(this.sortPositionMiddle); // Example initial position

    }

    //Start the two Shooter Motors
    public void setShooterPower(double power) {
        this.leftShooterMotor.setPower(power*0.01);
        this.rightShooterMotor.setPower(power*0.01);
    }
    public void setShooterVelocity(SIDES side) {
        if(this.usingPIDF) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            this.leftShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            this.rightShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }

        if (side == SIDES.LEFT) {
            this.leftShooterMotor.setVelocity(this.TARGET_VELOCITY_LEFT);
            this.rightShooterMotor.setVelocity(this.TARGET_VELOCITY_LEFT);
        } else {
            this.leftShooterMotor.setVelocity(this.TARGET_VELOCITY_RIGHT);
            this.rightShooterMotor.setVelocity(this.TARGET_VELOCITY_RIGHT);
        }
    }



    public void setIntakePower(double power) {
        this.intakeMotor.setPower(power);
    }
    public void setSorterServoPosition(double position) {
        sorterServo.setPosition(position);
    }
    public void setStoppers(boolean rp, boolean lp) {
        if(rp)  rightStopper.setPosition(rightStopperSTOP); else rightStopper.setPosition(rightStopperPASS);
        if(lp)  leftStopper.setPosition(leftStopperSTOP); else leftStopper.setPosition(leftStopperPASS);

    }
    public void indexerPower(double power, CRServo indexer) {
        indexer.setPower(power*0.01);
    }



    public void stopEverything(){
        this.setShooterPower(0);
        this.leftIndexerServo.setPower(0);
        this.rightIndexerServo.setPower(0);
        this.intakeMotor.setPower(0);
    }


    public boolean targetVelocityAcquired(SIDES side){
        if (side == SIDES.LEFT) {
            return leftShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_LEFT && rightShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_LEFT;
        }else if (side == SIDES.RIGHT){
            return leftShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_RIGHT && rightShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_RIGHT;
        }
        return leftShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_FIRST && rightShooterMotor.getVelocity() > TARGET_MIN_VELOCITY_FIRST;

    }
}