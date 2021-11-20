/* Copied from "https://www.w3schools.com/java/java_files_create.asp"
 *
 * Modified by team FIX IT 3491.
 */

package org.firstinspires.ftc.teamcode.Season_Setup;

import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors


public class WriteToFile extends Ducky {

    public static void allianceColour (String alliance) {
        try {
            FileWriter myWriter = new FileWriter("allianceColour.txt");
            myWriter.write(alliance);
            myWriter.close();
        } catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }
}