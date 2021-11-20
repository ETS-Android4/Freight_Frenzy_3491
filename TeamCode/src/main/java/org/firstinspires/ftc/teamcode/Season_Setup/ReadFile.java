/* Copied from "https://www.w3schools.com/java/java_files_read.asp"
 *
 * Modified by team FIX IT 3491.
 */

package org.firstinspires.ftc.teamcode.Season_Setup;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;


public class ReadFile extends Ducky {

    static File myObj = new File("allianceColour.txt");
    static Scanner myReader;

    static {
        try {
            myReader = new Scanner(myObj);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public boolean blueAlliance () {
        File myObj = new File("allianceColour.txt");
        String data = myReader.nextLine();
        if (data.equals("Red")) {
            blueAlliance = false;
        }

        return blueAlliance;
    }
}