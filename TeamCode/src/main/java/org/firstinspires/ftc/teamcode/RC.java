package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import org.firstinspires.ftc.robotcontroller.internal.GlobalValuesActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.firstinspires.ftc.teamcode.roboticslibrary.FXTTelemetry;


/**
 * Created by FIXIT on 15-08-21.
 */
public class RC {

    public static OpMode o;
    public static LinearOpMode l;
    //public static FXTTelemetry t;
    public static HardwareMap h;
    public static int runNum = 0;
    public final static String VUFORIA_LICENSE_KEY = "ATU9MNz/////AAABmdp9yZ8JdEGjpiGfxU8g64YjAQ" +
        "PwRcIIIqytyWu9HmjEkTELwI1JsCtkFv/I4k2S8KXjgWFB61R+GwLPvY3T1EyQmpV/UFfaSEqcJLpT++NbMjv5J" +
    "kXg3JG92Ga+RnHYS3WaTBgRZexhqar4QNK4exrzUQUJjy2ntF2Afb+ENqH4glLQW85aM0BA4+8WMjcplpZ5WbhJ82ruz0" +
    "ikcpy8bffFnhd+pN1/xficoB/Szcx5lt1SmKzVjbYkktmVd8qS6qGd8yVH1DydPQlP6njcUDllIc1a3oAO5zmWTFoxfaknDOm" +
                "2bXka6V2Qht6pD7pl1tSP3vgeCZPM0fKSowfy0MoFVzsuuBvwqloB4Obt4NDT";



    public static void setOpMode(OpMode op) {
        o = op;
        h = op.hardwareMap;
        //t = new FXTTelemetry();
        //t.setTelemetry(op.telemetry);

        if (op instanceof LinearOpMode) {
            l = (LinearOpMode) op;
        }//if

    }//setOpMode

    public static boolean globalBool(String key) {
        if (GlobalValuesActivity.globals.containsKey(key)) {
            return ((Boolean) GlobalValuesActivity.globals.get(key));
        }//if

        return false;
    }//globalBool

    public static String globalString(String key) {
        if (GlobalValuesActivity.globals.containsKey(key)) {
            return (String) GlobalValuesActivity.globals.get(key);
        }//if

        return "";
    }//globalString

    public static double globalDouble(String key) {
        if (GlobalValuesActivity.globals.containsKey(key)) {
            return ((Double) GlobalValuesActivity.globals.get(key));
        }//if

        return -1;
    }//globalDouble

    public static Object global(String key){
        return GlobalValuesActivity.globals.get(key);
    }

    public static String [] autoDashKeys(){
        String [] keys = new String[ GlobalValuesActivity.autoKeys.size()];
        GlobalValuesActivity.autoKeys.toArray(keys);
        return keys;
    }

    public static void setGlobalBool(String key, boolean val) {
        GlobalValuesActivity.add(key, val);
    }//globalBool

    public static void setGlobalString(String key, String val) {
        GlobalValuesActivity.add(key, val);
    }//globalString

    public static void setGlobalDouble(String key, double val) {
        GlobalValuesActivity.add(key, val);
    }//globalDouble


    public static Context c() {
        return AppUtil.getInstance().getActivity();
    }//context

    public static Activity a() {
        return (AppUtil.getInstance().getActivity());
    }//activity


    public static void stop() {
        //t.close();
    }//stop



}
