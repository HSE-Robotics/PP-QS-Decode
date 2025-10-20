package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.function.Supplier;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;


/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@TeleOp(name = "Reggie_TeleOP", group = "LM1 Reggie")
public class ReggieTeleop_v1 extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
    private Follower follower;
    private Reggie Miller;
    public static Pose startingPose;
    //Crush Variables

    public enum ScoringState{
        IDLE,
        ACCELERATING,
        SHOOTING,
        COOLING_DOWN
    }
    ScoringState artifactScoringState = ScoringState.IDLE;

    int shootingState = 0;
    public boolean inEndgame;
    public boolean blockpos;
    public ElapsedTime shootingTime, playTime;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        //Pedro Pathing
//        Constants.setConstants(FConstants.class, LConstants.class);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        Miller = new Reggie();
        Miller.init(hardwareMap);

        shootingTime = new ElapsedTime();
        playTime = new ElapsedTime();
        inEndgame = false;


       // playTime.reset();
    }
    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
        playTime.reset();
    }
    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        /**Pedro Pathing Driving
         *
         */
        follower.update();
        follower.setTeleOpDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

        switch (artifactScoringState){
            case IDLE:
                        if(gamepad1.left_bumper && shootingState==0){
                            shootingState=1;
                            Miller.setShooterPower(100,Miller.leftShooterMotor);
                            Miller.setShooterPower(100,Miller.rightShooterMotor);
                            setScoringState(ScoringState.ACCELERATING);


                        } else if (!gamepad1.left_bumper) {
                            shootingState = 0;
                        }
                        break;
            case ACCELERATING:
                        if(gamepad1.left_bumper && shootingState==0 && shootingTime.seconds()>3.0){
                            shootingState=1;
                            Miller.indexerPower(1.0,Miller.leftIndexerServo);
                            setScoringState(ScoringState.SHOOTING);
                        } else if (!gamepad1.left_bumper) {
                            shootingState = 0;
                        }
                        break;
            case SHOOTING:
                        if(shootingTime.seconds()>1.0 && gamepad1.left_bumper && shootingState==0){
                            Miller.indexerPower(0.0,Miller.leftIndexerServo);
                            Miller.setShooterPower(0.0,Miller.leftShooterMotor);
                            Miller.setShooterPower(0.0,Miller.rightShooterMotor);
                            shootingState=1;
                            setScoringState(ScoringState.IDLE);
                        }else if (!gamepad1.left_bumper) {
                            shootingState = 0;
                        }
                break;

        }
        if(gamepad1.left_trigger>0.125){
            Miller.setIntakePower(gamepad1.left_trigger, Miller.intakeMotor);
        } else if (gamepad1.right_trigger>0.125) {
            Miller.setIntakePower(gamepad1.right_trigger, Miller.intakeMotor);
        }else{
            Miller.setIntakePower(0.0, Miller.intakeMotor);
        }


        //General code for both options (1 or 2 Players)
        if(playTime.seconds()>40 && !inEndgame){
            gamepad1.rumble(3000);
            gamepad2.rumble(3000);
            endgameLED();
            inEndgame = true;
        }

        telemetry.addData("Play Time: ", playTime.seconds() );




    }

    public void endgameLED(){
        gamepad1.setLedColor(255,0,0,500);
        gamepad2.setLedColor(0,0,255,500);
    }
    public void setScoringState(ScoringState pState) {
        artifactScoringState = pState;
        shootingTime.reset();

    }

}
