package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlyWheelTuning extends OpMode {

    public double highVelocity = 1500;
    public double lowVelocity = 900;
    double  currentTargetVelocity =highVelocity;
    double P;
    double F;
    double[] stepSizes={10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;
    public DcMotorEx rightShooter;
    public DcMotorEx leftShooter;


    @Override
    public void init() {
        rightShooter = hardwareMap.get(DcMotorEx.class, "BS");
        leftShooter = hardwareMap.get(DcMotorEx.class, "TS");

        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        if(gamepad1.yWasPressed()){
            if(currentTargetVelocity==highVelocity){
                currentTargetVelocity = lowVelocity;
            }else{
                currentTargetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;

        }

        if(gamepad1.dpadLeftWasPressed()){
            F-=stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()){
            F+=stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            P+=stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()){
            P-=stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        rightShooter.setVelocity(currentTargetVelocity);
        leftShooter.setVelocity(currentTargetVelocity);

        double currentVelocityRight = rightShooter.getVelocity();
        double currentVelocityLeft = leftShooter.getVelocity();
        double leftVelError = currentTargetVelocity - currentVelocityLeft;
        double rightVelError = currentTargetVelocity - currentVelocityRight;

        telemetry.addData("Target Velocity", currentTargetVelocity);
        telemetry.addData("Right Current Velocity", currentVelocityRight);
        telemetry.addData("Right Error ", rightVelError);
        telemetry.addData("Left Current Velocity", currentVelocityLeft);
        telemetry.addData("Right Error ", rightVelError);
        telemetry.addData("Tuning P", "%.4f (D-pad U/D", P);
        telemetry.addData("Tuning F", "%.4f (D-pad L/R", F);
        telemetry.addData("Step Size", "%.4f (Button B", stepSizes[stepIndex]);

    }
}
