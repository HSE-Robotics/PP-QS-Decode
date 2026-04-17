package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class FacingMechanism {
    public Limelight3A limeLight;
    private Limelight3A limelight;
    private double llAngle;
    double Angle;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        limeLight = hwMap.get(Limelight3A.class,"limelight");

    }

    public void resetTimer(){
        timer.reset();
    }

    public double update(LLResult result){

        if (result == null || !result.isValid()) {
            return 0; // or return lastAngle, or return 0 to avoid crashing
        }

        llAngle = result.getDetectorResults().get(0).getTargetXDegrees();
        return llAngle;
    }

    public void switchPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }

}
