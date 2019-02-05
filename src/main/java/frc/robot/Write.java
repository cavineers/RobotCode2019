package frc.robot;
import java.io.File;
import java.io.PrintWriter;
 

public class Write {
   public static PrintWriter createCsvFile(String fileName) {
       PrintWriter pw = null;
       try{
           pw= new PrintWriter(new File(fileName));
       } catch(Exception e){
        //TODO
       }
       return pw;
   }

   public static void writeCsvFile(PrintWriter pw, int vel, int actual) {
       try {
              
              StringBuilder sb=new StringBuilder();
              String v = Integer.toString(vel);
              String a = Integer.toString(actual);
              sb.append(a);
              sb.append(",");
              sb.append(v);
              sb.append("\r\n");
              pw.write(sb.toString());
              } catch (Exception e) {
                 // TODO: handle exception
              }
   } 
}