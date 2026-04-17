package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {

    private DcMotor intake;
    private CRServo rightIndx;
    private CRServo leftIndx;

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        rightIndx = hwMap.get(CRServo.class, "rightIndx");
        rightIndx.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIndx = hwMap.get(CRServo.class, "leftIndx");
    }

    public void intakePower(Gamepad gamepad){

        if (gamepad.right_trigger > 0){
            intake.setPower(gamepad.right_trigger);
        }else if (gamepad.left_trigger > 0){
            intake.setPower(-gamepad.left_trigger);
        } else {
            intake.setPower(0);
        }
    }

}

