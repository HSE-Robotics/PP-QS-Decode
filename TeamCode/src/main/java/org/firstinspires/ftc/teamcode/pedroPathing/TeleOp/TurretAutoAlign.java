package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FacingMechanism;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;
import org.firstinspires.ftc.teamcode.pedroPathing.TurretMechanism;
import org.firstinspires.ftc.teamcode.pedroPathing.hoodAngle;
import org.firstinspires.ftc.teamcode.pedroPathing.intake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.Position;

@TeleOp

public class TurretAutoAlign extends OpMode {

    public Limelight3A limeLight;

    private Follower follower;
    private FacingMechanism turret = new FacingMechanism();
    private Reggie miller = new Reggie();
    private Servo hood;
    private DcMotor intake;
    private DcMotor ts;
    private DcMotor bs;
    private DcMotor indx;
    private CRServo rightIndx;
    private CRServo leftIndx;
    private Servo gate;



    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    public void init(){
        follower = Constants.createFollower(hardwareMap);
        limeLight = hardwareMap.get(Limelight3A.class,"limelight");
        turret.init(hardwareMap);
        miller.init(hardwareMap);
        hood = hardwareMap.get(Servo.class,"hood");
        intake = hardwareMap.get(DcMotor.class,"intake");
        bs = hardwareMap.get(DcMotor.class,"BS");
        ts = hardwareMap.get(DcMotor.class,"TS");
        ts.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIndx = hardwareMap.get(CRServo.class, "rightIndx");
        leftIndx = hardwareMap.get(CRServo.class, "leftIndx");
        rightIndx.setDirection(DcMotorSimple.Direction.REVERSE);
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);
        indx = hardwareMap.get(DcMotor.class,"indx");


    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        turret.resetTimer();
        limeLight.start();
        gate.setPosition(0.5);


    }


    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        //intake.intakePower(gamepad1);
        if (gamepad1.right_trigger > 0){
            intake.setPower(gamepad1.right_trigger * .75);
        }else if (gamepad1.left_trigger > 0){
            intake.setPower(-gamepad1.left_trigger * .75);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.dpadUpWasPressed()){
            miller.setShooterVelocity(Reggie.SIDES.LEFT);
            rightIndx.setPower(1.0);
            leftIndx.setPower(1.0);
        } else if (gamepad1.dpadLeftWasPressed()) {
            miller.setShooterVelocity(Reggie.SIDES.RIGHT);
            rightIndx.setPower(1.0);
            leftIndx.setPower(1.0);
        } else if (gamepad1.dpadDownWasPressed()) {
            miller.setShooterVelocity(Reggie.SIDES.FIRST);
            rightIndx.setPower(0.0);
            leftIndx.setPower(0.0);
        }
        if (gamepad1.ps) {
            gate.setPosition(0.85);
        }else{
            gate.setPosition(0.35);
        }


        LLResult result = limeLight.getLatestResult();
        ///turret

        //pipeline switch
        if (gamepad1.triangleWasPressed()){
            limeLight.pipelineSwitch(0);
        }
        if (gamepad1.crossWasPressed()){
            limeLight.pipelineSwitch(1);
        }

        if (gamepad1.leftBumperWasPressed()){
            hood.setPosition(hood.getPosition() - 0.1);
        }
        if (gamepad1.rightBumperWasPressed()){
            hood.setPosition(hood.getPosition() + 0.1);
        }

        if (gamepad1.circle){
            indx.setPower(1);
        }else if (gamepad1.square){
            indx.setPower(-0.5);
        }else {
            indx.setPower(0);
        }

        /// Telemetry
        if (result != null && result.isValid()) {
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ta", result.getTa());
        } else {
            telemetry.addLine("No Tag Detected");
        }
        telemetry.addData("hood",hood.getPosition());

    }
}
