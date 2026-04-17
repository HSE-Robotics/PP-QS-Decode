package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class hoodAngle {

    private Servo hood;

    // Tus valores reales
    private final double TA_MAX = 02.77;   // cerca
    private final double TA_MIN = 0.320;  // lejos
    private double lastHood = 0;
    public double servPos;


    public void init(HardwareMap hwMap) {
        hood = hwMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.FORWARD);
    }

    /*public void update(double ta) {

        if (ta <= 0.01) return;

        // Mapeo lineal
        double hoodPos = (ta - TA_MAX) / (TA_MIN - TA_MAX);
        hoodPos = Range.clip(hoodPos, 0.0, 1.0);

        double diff = hoodPos - lastHood;

        // Si el cambio es muy pequeño, no mover
        if (Math.abs(diff) < 0.01) {
            return;
        }

        // RATE LIMIT (máximo cambio por ciclo)
        double maxStep = 0.02;
        diff = Range.clip(diff, -maxStep, maxStep);

        hoodPos = lastHood + diff;

        // LOW-PASS FILTER (suavizado)
        hoodPos = 0.85 * lastHood + 0.15 * hoodPos;

        lastHood = hoodPos;

        hood.setPosition(hoodPos);
    }
*/
    public void servoLeft(boolean left){
        if (left){
            hood.setPosition(hood.getPosition() - 0.1);
        }
    }
    public void servoRight(boolean right){
        if (right){
            hood.setPosition(hood.getPosition() - 0.1);
        }
    }

    public double servoPos(){
        servPos = hood.getPosition();
        return servoPos();
    }
}