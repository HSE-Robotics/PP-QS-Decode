package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Reggie;


/**
 *
 * @author Gerry DLIII - 18908 Mighty Hawks
 * @version 1.0, 02/11/2024
 */
@TeleOp(name = "Reggie_TeleOP2", group = "LM3/Tournament Reggie")
public class ReggieTeleop_v2 extends OpMode {
    //private static final Logger log = LoggerFactory.getLogger(AyCrush2P_PP.class);
    //Pedro Pathing Variables
    private ColorSensor colorSensorLeft;
    public HuskyLens huskyLens;
    private ColorSensor colorSensorRight;
    private Follower follower;
    private Reggie Miller;
    public static Pose startingPose = Reggie.poseFromAuto;
    public static Pose scoringPose = Reggie.scoringPose;

    public enum ScoringState {
        IDLE,
        ACCELERATING_FAR,
        ACCELERATING_NEAR,
        SHOOTING_RIGHT,
        SHOOTING_LEFT,
        COOLING_DOWN
    }

    ScoringState artifactScoringState = ScoringState.IDLE;
    int longDistanceVelocity = 1500;
    int midDistanceVelocity = 1300;
    int closeDistanceVelocity = 1150;
    int longDistancePower = 90;
    int midDistancePower = 68;
    int closeDistancePower = 60;

    int shootingState = 0;
    public boolean inEndgame;
    public ElapsedTime shootingTime, playTime;
    double sortPositionMiddle =0.5 ;
    double sortPositionRight = 0.05;
    double sortPositionLeft = 0.9;
    int Side = 0;
    int rRed;
    int rGreen;
    int rBlue;
    int lRed;
    int lGreen;
    int lBlue;


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

        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

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

        lBlue = colorSensorLeft.blue();
        lGreen = colorSensorLeft.green();
        lRed = colorSensorLeft.red();
        rBlue = colorSensorRight.blue();
        rGreen = colorSensorRight.green();
        rRed = colorSensorRight.red();




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
            Miller.setIntakePower(-gamepad1.left_trigger);
            

        } else if (gamepad1.right_trigger > 0.125) {
            Miller.setIntakePower(gamepad1.right_trigger);
            Miller.setStoppers(true,true);
            Miller.setSorterServoPosition(.7);
        } else {
            Miller.setIntakePower(0.0);
        }


        //Indexers Code
        if (gamepad1.right_bumper) {
            Miller.indexerPower(80, Miller.rightIndexerServo);
            Miller.indexerPower(0, Miller.leftIndexerServo);
            Miller.setSorterServoPosition(Miller.sortPositionRight);
            Miller.setStoppers(false,true);
            Miller.setIntakePower(0.4);
        } else if (gamepad1.left_bumper) {
            Miller.indexerPower(80, Miller.leftIndexerServo);
            Miller.indexerPower(0, Miller.rightIndexerServo);
            Miller.setSorterServoPosition(Miller.sortPositionLeft);
            Miller.setStoppers(true,false);
            Miller.setIntakePower(0.4);
        } else {
            Miller.indexerPower(0, Miller.leftIndexerServo);
            Miller.indexerPower(0, Miller.rightIndexerServo);
        }

        //Flywheels
        if (gamepad1.square) {
            Miller.TARGET_MIN_VELOCITY_LEFT = longDistanceVelocity - 100;
            Miller.TARGET_VELOCITY_LEFT = longDistanceVelocity;
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
//            Miller.setShooterPower(longDistancePower);
        } else if (gamepad1.triangle) {

            Miller.TARGET_MIN_VELOCITY_LEFT = midDistanceVelocity - 100;
            Miller.TARGET_VELOCITY_LEFT = midDistanceVelocity;
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
//            Miller.setShooterPower(midDistancePower);
        } else if (gamepad1.circle) {
//            Miller.setShooterPower(closeDistancePower);

            Miller.TARGET_MIN_VELOCITY_LEFT = closeDistanceVelocity - 100;
            Miller.TARGET_VELOCITY_LEFT = closeDistanceVelocity;
            Miller.setShooterVelocity(Reggie.SIDES.LEFT);
//            Miller.setShooterVelocity(closeDistanceVelocity);
        } else if (gamepad1.cross) {
            Miller.setShooterPower(0);
        }

        //Sorter
        if (gamepad1.dpad_left) {
            Miller.setSorterServoPosition(Miller.sortPositionLeft);
        } else if (gamepad1.dpad_right) {
            Miller.setSorterServoPosition(Miller.sortPositionRight);
        } else if (gamepad1.dpad_down) {
            Miller.setSorterServoPosition(sortPositionMiddle);
        }

        //General code for both options (1 or 2 Players)
        if (playTime.seconds() > 40 && !inEndgame) {
            //gamepad1.rumble(3000);
            //gamepad2.rumble(3000);
            endgameLED();
            inEndgame = true;
        }

        if(gamepad1.psWasPressed()){
            Pose currentPose = follower.getPose();
             PathChain ScoringPath;
            ScoringPath = follower.pathBuilder().
                            addPath(new BezierLine(currentPose, scoringPose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(),scoringPose.getHeading())
                                            .build();
            follower.followPath(ScoringPath, 0.85,true);
        }

        /*
        if(gamepad1.dpad_down){
            Miller.leftStopper.setPosition(Miller.leftStopper.getPosition()-0.01);
            Miller.rightStopper.setPosition(Miller.rightStopper.getPosition()-0.01);
        }else if(gamepad1.dpad_up){
            Miller.leftStopper.setPosition(Miller.leftStopper.getPosition()+0.01);
            Miller.rightStopper.setPosition(Miller.rightStopper.getPosition()+0.01);
        }
        */

        /*else if(gamepad1.dpad_up){
            Miller.setStoppers(false, true);
        }else if(gamepad1.dpad_left){
            Miller.setStoppers(true, false);
        }else if(gamepad1.dpad_right){
            Miller.setStoppers(true, true);
        }*/

        telemetry.addData("Side", Side);

        telemetry.addData("Play Time: ", playTime.seconds());
        if (artifactScoringState == ScoringState.ACCELERATING_FAR) {
            telemetry.addData("Its Miller Time - ", "Shooting from Far Away!");
        }
        telemetry.addData("Current State:", artifactScoringState.toString());

        telemetry.addData("Left Blue:",lBlue);
        telemetry.addData("Left Green:",lGreen);
        telemetry.addData("Left Red:",lRed);
        telemetry.addData("Right Blue:",rBlue);
        telemetry.addData("Right Green:",rGreen);
        telemetry.addData("Right Red:",rRed);
        telemetry.addData("Left Stopper Position:",Miller.leftStopper.getPosition());
        telemetry.addData("Right Stopper Position:",Miller.rightStopper.getPosition());
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
