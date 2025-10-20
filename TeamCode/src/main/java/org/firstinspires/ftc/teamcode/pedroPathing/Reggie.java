package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Reggie {

    // Declare hardware components as public to be accessible in OpModes
    public DcMotor intakeMotor = null;
    public DcMotor leftShooterMotor = null;
    public DcMotor rightShooterMotor = null;
    public Servo sorterServo = null;
    public CRServo leftIndexerServo = null;
    public CRServo rightIndexerServo = null;

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

        intakeMotor = hwMap.get(DcMotor.class, "int");
        leftShooterMotor = hwMap.get(DcMotor.class, "LS");
        rightShooterMotor = hwMap.get(DcMotor.class, "RS");

        // Define and Initialize Servos
        sorterServo = hwMap.get(Servo.class, "indServo");
        leftIndexerServo = hwMap.get(CRServo.class, "LServo");
        rightIndexerServo = hwMap.get(CRServo.class, "RServo");

        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set all motors to zero power
        intakeMotor.setPower(0);
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);

        // Set all motors to run without encoders
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set servo initial position
        this.setSorterServoPosition(0.25); // Example initial position
    }


    public void setShooterPower(double power, DcMotor shooter) {
        shooter.setPower(power);
    }
    public void setIntakePower(double power, DcMotor Intake) {
        Intake.setPower(power);
    }
    public void setSorterServoPosition(double position) {
        sorterServo.setPosition(position);
    }
    public void indexerPower(double power, CRServo indexer) {indexer.setPower(power);}
}