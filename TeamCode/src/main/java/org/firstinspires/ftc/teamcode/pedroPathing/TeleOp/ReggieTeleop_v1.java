package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.function.Supplier;
import com.qualcomm.hardware.dfrobot.HuskyLens;

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

    public enum ScoringState {
        IDLE,
        ACCELERATING_FAR,
        ACCELERATING_NEAR,
        SHOOTING_RIGHT,
        SHOOTING_LEFT,
        COOLING_DOWN
    }

    ScoringState artifactScoringState = ScoringState.IDLE;
    int longDistancePower = 90;
    int midDistancePower = 68;
    int closeDistancePower = 60;

    int shootingState = 0;
    public boolean inEndgame;
    public ElapsedTime shootingTime, playTime;

    double sortPositionMiddle = 0.45;
    double sortPositionRight = 0.25;
    double sortPositionLeft = 0.75;
    int Side = 0;


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

    /**
     * This method is called once at the start of the OpMode.
     **/
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
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        /*
        switch (artifactScoringState){
            case IDLE:
                        if(gamepad1.left_bumper && shootingState==0){
                            shootingState=1;
                            Miller.setShooterPower(90,Miller.leftShooterMotor);
                            Miller.setShooterPower(90,Miller.rightShooterMotor);
                            setScoringState(ScoringState.ACCELERATING_FAR);


                        } else if (!gamepad1.left_bumper) {
                            shootingState = 0;
                        }
                        if(gamepad1.right_bumper && shootingState==0){
                            shootingState=1;
                            Miller.setShooterPower(80,Miller.leftShooterMotor);
                            Miller.setShooterPower(80,Miller.rightShooterMotor);
                            setScoringState(ScoringState.ACCELERATING_NEAR);


                        } else if (!gamepad1.right_bumper) {
                            shootingState = 0;
                        }
                        break;
            case ACCELERATING_FAR:
                        if(gamepad1.left_bumper && shootingState==0 && shootingTime.seconds()>3.0){
                            shootingState=1;
                            Miller.indexerPower(90,Miller.leftIndexerServo);
                            setScoringState(ScoringState.SHOOTING_LEFT);
                        }else if(gamepad1.right_bumper && shootingState==0 && shootingTime.seconds()>3.0){
                            shootingState=1;
                            Miller.indexerPower(90,Miller.rightIndexerServo);
                            setScoringState(ScoringState.SHOOTING_RIGHT);
                        }else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                            shootingState = 0;
                        }
                        break;
            case ACCELERATING_NEAR:
                        if(gamepad1.left_bumper && shootingState==0 && shootingTime.seconds()>3.0){
                            shootingState=1;
                            Miller.indexerPower(80,Miller.leftIndexerServo);
                            setScoringState(ScoringState.SHOOTING_LEFT);
                        }else if(gamepad1.right_bumper && shootingState==0 && shootingTime.seconds()>3.0){
                            shootingState=1;
                            Miller.indexerPower(80,Miller.rightIndexerServo);
                            setScoringState(ScoringState.SHOOTING_RIGHT);
                        }else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                            shootingState = 0;
                        }
                        break;
            case SHOOTING_LEFT:
                        if(shootingTime.seconds()>1.0 && gamepad1.left_bumper && shootingState==0){
                            Miller.indexerPower(0.0,Miller.leftIndexerServo);
                            Miller.setShooterPower(0.0,Miller.leftShooterMotor);
                            Miller.setShooterPower(0.0,Miller.rightShooterMotor);
                            shootingState=0;
                            setScoringState(ScoringState.IDLE);
                        }else if (!gamepad1.left_bumper) {
                            shootingState = 0;
                        }
                break;

        }
        */

        //Intake
        if (gamepad1.left_trigger > 0.125) {
            Miller.setIntakePower(-gamepad1.left_trigger, Miller.intakeMotor);
        } else if (gamepad1.right_trigger > 0.125) {
            Miller.setIntakePower(gamepad1.right_trigger, Miller.intakeMotor);
        } else {
            Miller.setIntakePower(0.0, Miller.intakeMotor);
        }


        //Indexers Code
        if (gamepad1.right_bumper) {
            Miller.indexerPower(85, Miller.rightIndexerServo);
            Miller.indexerPower(0, Miller.leftIndexerServo);
            Miller.setSorterServoPosition(sortPositionRight);
        } else if (gamepad1.left_bumper) {
            Miller.indexerPower(85, Miller.leftIndexerServo);
            Miller.indexerPower(0, Miller.rightIndexerServo);
            Miller.setSorterServoPosition(sortPositionLeft);
        } else {
            Miller.indexerPower(0, Miller.leftIndexerServo);
            Miller.indexerPower(0, Miller.rightIndexerServo);
        }

        //Flywheels
        if (gamepad1.square) {
            Miller.setShooterPower(longDistancePower);
        } else if (gamepad1.triangle) {
            Miller.setShooterPower(midDistancePower);
        } else if (gamepad1.circle) {
            Miller.setShooterPower(closeDistancePower);
        } else if (gamepad1.cross) {
            Miller.setShooterPower(0);
        }

        //Sorter
        if (gamepad1.dpad_left) {
            Miller.setSorterServoPosition(sortPositionLeft);
        } else if (gamepad1.dpad_right) {
            Miller.setSorterServoPosition(sortPositionRight);
        } else if (gamepad1.dpad_down) {
            Miller.setSorterServoPosition(sortPositionMiddle);
        }

        //General code for both options (1 or 2 Players)
        if (playTime.seconds() > 40 && !inEndgame) {
            gamepad1.rumble(3000);
            gamepad2.rumble(3000);
            endgameLED();
            inEndgame = true;
        }

        HuskyLens.Block[] blocks = Miller.huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
            if (blocks[i].id == 1) {
                telemetry.addLine("RED");
                Side = 1;
            }
            if (blocks[i].id == 2) {
                telemetry.addLine("BLUE");
                Side = 2;
            }
            if (!(blocks[i].id == 1 || blocks[i].id == 2)){
                telemetry.addLine("NONE");
                Side = 0;
            }
            telemetry.addData("Side", Side);

            telemetry.addData("Play Time: ", playTime.seconds());
            if (artifactScoringState == ScoringState.ACCELERATING_FAR) {
                telemetry.addData("Its Miller Time - ", "Shooting from Far Away!");
            }
            telemetry.addData("Current State:", artifactScoringState.toString());

        }

        }
    public void endgameLED () {
        gamepad1.setLedColor(255, 0, 0, 500);
        gamepad2.setLedColor(0, 0, 255, 500);
    }
    public void setScoringState (ScoringState pState){
        artifactScoringState = pState;
        shootingTime.reset();

    }

}
