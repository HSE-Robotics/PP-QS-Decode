package org.firstinspires.ftc.teamcode.pedroPathing;


import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

public class Reggie {

    // Declare hardware components as public to be accessible in OpModes
    public DcMotor intakeMotor = null;
    public DcMotor leftShooterMotor = null;
    public DcMotor rightShooterMotor = null;
    public Servo sorterServo = null;
    public CRServo leftIndexerServo = null;
    public CRServo rightIndexerServo = null;
    public HuskyLens huskyLens;

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

        huskyLens = hwMap.get(HuskyLens.class, "huskylens");

        leftShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightIndexerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set all motors to zero power
        intakeMotor.setPower(0);
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);

        // Set all motors to run without encoders
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // Set servo initial position
        this.setSorterServoPosition(0.25); // Example initial position

    }

    public void setShooterPower(double power) {
        this.leftShooterMotor.setPower(power*0.01);
        this.rightShooterMotor.setPower(power*0.01);
    }
    public void setIntakePower(double power, DcMotor Intake) {
        Intake.setPower(power);
    }
    public void setSorterServoPosition(double position) {
        sorterServo.setPosition(position);
    }
    public void indexerPower(double power, CRServo indexer) {indexer.setPower(power*0.01);}

    public int aim(HuskyLens hl) {
        int Side = 0;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            if (blocks[i].id == 1) {
                //RED
                Side = 1;
            }
            if (blocks[i].id == 2) {
                //Blue
                Side = 2;
            }

            return Side;
            /*
             * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
             * - blocks[i].width and blocks[i].height   (size of box, in pixels)
             * - blocks[i].left and blocks[i].top       (edges of box)
             * - blocks[i].x and blocks[i].y            (center location)
             * - blocks[i].id                           (Color ID)
             *
             * These values have Java type int (integer).
             */
        }

        return Side;
    }
    public void shoot(double power){

    }



}