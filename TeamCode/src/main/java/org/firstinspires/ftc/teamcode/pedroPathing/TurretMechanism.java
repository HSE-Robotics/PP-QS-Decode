package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class TurretMechanism {
    public Limelight3A limeLight;
    private DcMotor encoder;
    private Limelight3A limelight;
    private double kP = 0.019;
    private double kD = 0.0015;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.25;
    private final double MAX_POWER = 1;
    private double power = 0;
    private double ticksTraveled;
    private boolean teleportRight = false;
    private boolean teleportLeft = false;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        encoder = hwMap.get(DcMotor.class,"turret");
        encoder.setDirection(DcMotorSimple.Direction.FORWARD);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limeLight = hwMap.get(Limelight3A.class,"limelight");

    }
    public void setkP(double newKP){
        kP = newKP;
    }

    public double getkP(){
        return kP;
    }
    public void setkD(double newKD){
        kD = newKD;
    }

    public double getkD(){
        return kD;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void update(LLResult result){

        ticksTraveled = encoder.getCurrentPosition();
        double deltaTime = timer.seconds();
        timer.reset();

        boolean atRightLimit = ticksTraveled > 6400;
        boolean atLeftLimit  = ticksTraveled < -6400;
        boolean atRightLimitTele = ticksTraveled > 5000;
        boolean atLeftLimitTele  = ticksTraveled < -5000;

        // Teleport Right
        if (teleportRight) {

            encoder.setPower(0.5); // to Right

            if (atRightLimitTele) {
                encoder.setPower(0);
                teleportRight = false;
            }

            return;
        }

        // Teleport Left
        if (teleportLeft) {

            encoder.setPower(-0.5); // to Left

            if (atLeftLimitTele) {
                encoder.setPower(0);
                teleportLeft = false;
            }

            return;
        }

        // No target, no teleport
        if (result == null || !result.isValid()) {
            encoder.setPower(encoder.getPower() * 0.85);
            lastError = 0;
            return;
        }
        // Checks if At Limit
        if (atLeftLimit) {
            teleportRight = true;
            return;
        }

        if (atRightLimit) {
            teleportLeft = true;
            return;
        }

        // Tracking Math
        double tx = result.getTx();

        if (Math.abs(tx) < 0.1) {
            tx = 0;
        }
        double error = goalX - tx;

        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0){
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        double rawPower = pTerm + dTerm;
        double limitedPower = Range.clip(rawPower, -MAX_POWER, MAX_POWER);

        if (Math.abs(error) < angleTolerance) {
            limitedPower = 0;
        }

        encoder.setPower(limitedPower);
        lastError = error;
    }
    public void switchPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }

    public double encoder(){
        ticksTraveled = encoder.getCurrentPosition();
        return ticksTraveled;
    }
}
