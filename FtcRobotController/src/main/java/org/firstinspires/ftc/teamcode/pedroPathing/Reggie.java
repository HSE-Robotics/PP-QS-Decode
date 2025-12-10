package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;


public class Reggie {

    // Declare hardware components as public to be accessible in OpModes
    private Timer pathTimer;
    public DcMotor intakeMotor = null;
    public DcMotorEx leftShooterMotor = null;
    public DcMotorEx rightShooterMotor = null;
    public Servo sorterServo = null;
    public Servo rightStopper = null;
    public Servo leftStopper = null;
    public CRServo leftIndexerServo = null;
    public CRServo rightIndexerServo = null;
    public HuskyLens huskyLens;
    public ElapsedTime idlerTime;

    public final double TARGET_VELOCITY = 1250;
    public final double TARGET_MIN_VELOCITY = 1100;

    public double sortPositionMiddle = 0.45;
    public double sortPositionRight = 0.05;
    public double sortPositionLeft = 0.9;

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
        leftShooterMotor = hwMap.get(DcMotorEx.class, "LS");
        rightShooterMotor = hwMap.get(DcMotorEx.class, "RS");

        // Define and Initialize Servos
        sorterServo = hwMap.get(Servo.class, "indServo");
        rightStopper = hwMap.get(Servo.class, "RStopper");
        leftStopper = hwMap.get(Servo.class, "LStopper");
        leftIndexerServo = hwMap.get(CRServo.class, "LServo");
        rightIndexerServo = hwMap.get(CRServo.class, "RServo");

        huskyLens = hwMap.get(HuskyLens.class, "huskylens");
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
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);


        // Set servo initial position
        this.setSorterServoPosition(this.sortPositionMiddle); // Example initial position

    }

    public void setShooterPower(double power) {
        this.leftShooterMotor.setPower(power*0.01);
        this.rightShooterMotor.setPower(power*0.01);
    }
    public void setShooterVelocity(double velocity) {
        this.leftShooterMotor.setVelocity(velocity);
        this.rightShooterMotor.setVelocity(velocity);
    }

    public void setIntakePower(double power, DcMotor Intake) {
        Intake.setPower(power);
    }
    public void setSorterServoPosition(double position) {
        sorterServo.setPosition(position);
    }
    public void setStoppers(double rp, double lp) {
        rightStopper.setPosition(rp);
        leftStopper.setPosition(lp);
    }
    public void indexerPower(double power, CRServo indexer) {
        indexer.setPower(power*0.01);
    }

    public boolean aim(HuskyLens hl) {
        int Side = 0;
        boolean fire = false;
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
            if ((blocks[i].id == 1 || blocks[i].id == 2) && (blocks[i].width >= 44 && blocks[i].width <= 52 && blocks[i].height >= 44 && blocks[i].height <= 52))
                fire = true;
            else if (!(blocks[i].id == 1 || blocks[i].id == 2)){
                fire = false;
            }
        }

        return fire;
    }
    public int order(HuskyLens hl) {
        int orden = 0;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block block : blocks) {
            if (block.id == 3) {
                //PPG
                orden = 1;
            }
            if (block.id == 4) {
                //GPP
                orden = 2;
            }
            if (block.id == 5) {
                //PGP
                orden = 3;

            } else if (block.width != 44 && block.height != 44) {
            }
        }

        return orden;
    }


    public void PPG(){
        indexerPower(70, leftIndexerServo);
        setIntakePower(100,intakeMotor);
        setSorterServoPosition(1);
        if(pathTimer.getElapsedTimeSeconds() >=5){
            setSorterServoPosition(0);
            indexerPower(80, rightIndexerServo);
            if (pathTimer.getElapsedTimeSeconds() >= 7){
                indexerPower(0, rightIndexerServo);
                indexerPower(0, leftIndexerServo);
                setShooterPower(0);
                setIntakePower(0,intakeMotor);
            }
        }
    }

    public void shootClose(){

        setShooterPower(60);

        if (order(huskyLens) == 1) {
            PPG();
        }


    }

    public void stopEverything(){
        this.setShooterPower(0);
        this.leftIndexerServo.setPower(0);
        this.rightIndexerServo.setPower(0);
        this.intakeMotor.setPower(0);
    }
}