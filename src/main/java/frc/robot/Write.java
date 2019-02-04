package frc.robot;
import java.io.FileWriter;
import java.io.IOException;
 

public class Write {
     
    //Delimiter used in CSV file
    private static final String COMMA_DELIMITER = ",";
    private static final String NEW_LINE_SEPARATOR = "\n";
     
    //CSV file header
    private static final String FILE_HEADER = "VelTrap,Actual";
 
    public static void writeCsvFile(String fileName, int vel, int actual) {
        FileWriter fileWriter = null;
                 
        try {
            fileWriter = new FileWriter(fileName);
 
            //Write the CSV file header
            fileWriter.append(FILE_HEADER.toString());
             
            //Add a new line separator after the header
            fileWriter.append(NEW_LINE_SEPARATOR);

            String v = Integer.toString(vel);
            String a = Integer.toString(actual);
            fileWriter.append(v);
            fileWriter.append(COMMA_DELIMITER);
            fileWriter.append(a);
            fileWriter.append(NEW_LINE_SEPARATOR);
             
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
             
            try {
                fileWriter.flush();
                fileWriter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
             
        }
    }
}