
/**
 * Class represents SCARA robotic arm.
 *
 * @Arthur Roberts
 * @0.0
 */

import ecs100.UI;
import java.awt.Color;
import java.util.*;
import java.io.*;
import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;

public class Arm
{

    // fixed arm parameters
    public static final int XM1 = 287;  // coordinates of the motor(measured in pixels of the picture)
    public static final int YM1 = 374;
    public static final int XM2 = 377;
    public static final int YM2 = 374;
    public static final double R = 154;  // length of the upper/fore arm
    public static final double D = XM2-XM1;
    public static final double YMIN = YM1+(YM2-YM1)/2 - Math.sqrt(R*R - Math.pow(R-XM1+(XM2-XM1)/2,2));
    public static final double YMAX = YMIN - (Math.sqrt(3*R*R) - YM2 + YMIN);
    // parameters of servo motors - linear function pwm(angle)
    // each of two motors has unique function which should be measured
    // linear function cam be described by two points
    // motor 1, point1
    private double pwm1_val_1;
    private double theta1_val_1;
    // motor 1, point 2
    private double pwm1_val_2;
    private double theta1_val_2;

    // motor 2, point 1
    private double pwm2_val_1;
    private double theta2_val_1;
    // motor 2, point 2
    private double pwm2_val_2;
    private double theta2_val_2;

    // current state of the arm
    private double theta1; // angle of the upper arm
    private double theta2;

    private double xj1;
    private double yj1;
    private double xj12;
    private double yj12;

    private double xj2;
    private double yj2;
    private double xj22;
    private double yj22;

    private double xt;     // position of the tool
    private double yt;
    private double xt2;
    private double yt2;

    private boolean valid_state; // is state of the arm physically possible?

    private boolean[][] photo;

    /**
     * Constructor for objects of class Arm
     */
    public Arm()
    {
        theta1 = -90.0*Math.PI/180.0; // initial angles of the upper arms
        theta2 = -90.0*Math.PI/180.0;
        valid_state = false;
    }

    public void loadPhoto(String filename) {
        try {
            BufferedImage img = ImageIO.read(new File(filename));
            int rows = img.getHeight();
            int cols = img.getWidth();
            photo = new boolean[rows][cols];
            for (int row = 0; row < rows; row++){
                for (int col = 0; col < cols; col++){
                    photo[row][col] = img.getRGB(col, row)<-200000;
                }
            }
            UI.printMessage("Loaded "+ filename);
            processPhoto();
        } catch(IOException e){UI.printf("/nImage reading failed: %s/n",e);}
    }

    private void processPhoto(){

    }

    public boolean[][] getPhoto(){
        return photo;
    }

    // draws arm on the canvas
    public void draw()
    {
        // draw arm
        int height = UI.getCanvasHeight();
        int width = UI.getCanvasWidth();
        // calculate joint positions
        //xj1 = XM1 + R*Math.cos(theta1);
        //yj1 = YM1 + R*Math.sin(theta1);
        //xj2 = XM2 + R*Math.cos(theta2);
        //yj2 = YM2 + R*Math.sin(theta2);

        //draw motors and write angles
        int mr = 20;
        UI.setLineWidth(5);
        UI.setColor(Color.BLUE);
        UI.drawOval(XM1-mr/2,YM1-mr/2,mr,mr);
        UI.drawOval(XM2-mr/2,YM2-mr/2,mr,mr);
        // write parameters of first motor
        String out_str=String.format("t1=%3.1f",theta1);
        UI.drawString(out_str, XM1-2*mr,YM1-mr/2+2*mr);
        out_str=String.format("XM1=%d",XM1);
        UI.drawString(out_str, XM1-2*mr,YM1-mr/2+3*mr);
        out_str=String.format("YM1=%d",YM1);
        UI.drawString(out_str, XM1-2*mr,YM1-mr/2+4*mr);
        UI.drawString("PWM1="+Double.toString(get_pwm1()), XM1-2*mr,YM1-mr/2+5*mr);
        // ditto for second motor
        out_str = String.format("t2=%3.1f",theta2);
        UI.drawString(out_str, XM2+2*mr,YM2-mr/2+2*mr);
        out_str=String.format("XM2=%d",XM2);
        UI.drawString(out_str, XM2+2*mr,YM2-mr/2+3*mr);
        out_str=String.format("YM2=%d",YM2);
        UI.drawString(out_str, XM2+2*mr,YM2-mr/2+4*mr);
        UI.drawString("PWM2="+Double.toString(get_pwm2()), XM2+2*mr,YM2-mr/2+5*mr);
        // draw Field Of View
        UI.setColor(Color.GRAY);
        UI.drawRect(0,0,640,480);
        UI.drawRect(XM1+D/2-R/2,YMAX,R,(YMAX-YMIN)*-1);
        //UI.println(Double.toString(R) + " BY " + Double.toString((YMAX-YMIN)*-1));

        // it can b euncommented later when
        // kinematic equations are derived
        if ( valid_state) {
            UI.printMessage("");
            // draw upper arms
            UI.setColor(Color.GREEN);
            UI.drawLine(XM1,YM1,xj1,yj1);
            UI.drawLine(XM2,YM2,xj2,yj2);
            //draw forearms
            UI.drawLine(xj1,yj1,xt,yt);
            UI.drawLine(xj2,yj2,xt,yt);
            // draw tool
            double rt = 20;
            UI.drawOval(xt-rt/2,yt-rt/2,rt,rt);
        }

    }

    // calculate tool position from motor angles
    // updates variable in the class
    public void directKinematic(double t1, double t2){
        theta1 = Math.toRadians(t1);
        theta2 = Math.toRadians(t2);

        xj1 = XM1 + R*Math.cos(theta1);
        yj1 = YM1 + R*Math.sin(theta1);
        xj2 = XM2 + R*Math.cos(theta2);
        yj2 = YM2 + R*Math.sin(theta2);

        if (xj1 > xj2 || yj1 > YM1 || yj2 > YM2){fail("Broked"); return;}

        // midpoint between joints
        double  xa = xj1+(xj2-xj1)/2;
        double  ya = yj1+(yj2-yj1)/2;

        // distance between joints
        double d = Math.sqrt(Math.pow(xj2-xj1,2) + Math.pow(yj2-yj1,2));

        if (d>2*R){fail("Broked"); return;}

        // half distance between tool positions
        double  h = Math.sqrt( (R*R) - Math.pow((0.5*d),2) );
        double alpha = Math.atan(Math.abs((yj1-yj2)/(xj2-xj1))*-1);

        // Valid Case
        valid_state = true;
        xt = xa + h * Math.cos((Math.PI/2) - alpha);
        yt = ya - h * Math.sin((Math.PI/2) - alpha);
        xt2 = xa - h*Math.cos(Math.PI/2 - alpha);
        yt2 = ya - h*Math.sin(Math.PI/2 - alpha); 
    }

    private void fail(String failMessage){
        valid_state = false;
        UI.printMessage(failMessage);
    }

    //motor angles from tool position
    //updates variables of the class
    public void inverseKinematic(double xt_new,double yt_new){
        valid_state = true;
        xt = xt_new;
        yt = yt_new;
        // distance between pwm and motor
        double d1 = (Math.sqrt(Math.pow(xt-XM1,2) + Math.pow(yt-YM1,2)))/2; //Half distance from t to M1 for trig
        double d2 = (Math.sqrt(Math.pow(XM2-xt,2) + Math.pow(YM2-yt,2)))/2; //Half distance from t to M2 for trig
        double h1 = Math.sqrt(Math.pow(R,2) - Math.pow(d1,2));
        double h2 = Math.sqrt(Math.pow(R,2) - Math.pow(d2,2));

        //Invalid Checks For Angles
        if (d1*2>2*R || d2*2>2*R || yt > YMIN){
            valid_state = false;
            if (d1*2>2*R && d2*2>2*R)fail("Motors Can't Reach");
            else if (d1>2*R)fail("Motor 1 Can't Reach");
            else if(d2>2*R)fail("Motor 2 Can't Reach");
            else if (yt > YMIN) fail("Less than YMIN");
            return;
        }

        double xa1 = XM1 + (xt-XM1)/2;
        double ya1 = YM1 + (yt-YM1)/2;
        double xa2 = xt + (XM2-xt)/2;
        double ya2 = yt + (YM2-yt)/2;

        double alpha1 = Math.atan2((YM1-yt),(xt-XM1));
        double alpha2 = Math.atan2((YM2-yt),(xt-XM2));

        xj1 = xa1 - h1*Math.cos(Math.PI/2-alpha1);
        yj1 = ya1 - h1*Math.sin(Math.PI/2-alpha1);
        xj12 = xa1 + h1*Math.cos(Math.PI/2-alpha1);
        yj12 = ya1 + h1*Math.sin(Math.PI/2-alpha1);

        xj2 = xa2 + h2*Math.cos(Math.PI/2-alpha2);
        yj2 = ya2 + h2*Math.sin(Math.PI/2-alpha2);
        xj22 = xa2 - h2*Math.cos(Math.PI/2-alpha2);
        yj22 = ya2 - h2*Math.sin(Math.PI/2-alpha2);

        if (xj1 == XM1){
            theta1 = 90;
        } else {
            theta1 = (xj1 < XM1) ? -180+Math.toDegrees(Math.asin((YM1-yj1)/R)) : -Math.toDegrees(Math.asin((YM1-yj1)/R));
        }
        if (xj2 == XM2){
            theta2 = 90;
        } else {
            theta2 = (xj2 < XM2) ? -180+(Math.toDegrees(Math.asin((YM2-yj2)/R))) : -Math.toDegrees(Math.asin((YM2-yj2)/R));
        }

        if ((theta1 > 0)||(theta1 < -180) || Double.isNaN(theta1)){
            valid_state = false;
            fail("Angle 1 - invalid");
            return;
        }
        if ((theta2 > 0)||(theta2 < -180) || Double.isNaN(theta2)){
            valid_state = false;
            fail("Angle 2 - invalid");
            return;
        }
    }

    // returns angle of motor 1
    public double get_theta1(){
        return theta1;
    }
    // returns angle of motor 2
    public double get_theta2(){
        return theta2;
    }
    // sets angle of the motors
    public void set_angles(double t1, double t2){
        theta1 = t1;
        theta2 = t2;
    }

    // returns motor control signal
    // for motor to be in position(angle) theta1
    // linear intepolation
    public int get_pwm1(){
        //double pwm = -11.226688*theta1 + 504.736842;
        double pwm = -10.074982*theta1 + 241.633131;
        //double pwm = -10*theta1 + 200;

        return (int)pwm;
    }
    // ditto for motor 2
    public int get_pwm2(){
        //double pwm = -10.313073*theta2 + 708.347805;
        //double pwm = -10*theta2 + 1000;
        double pwm = -9.6437354*theta2 + 920.702246;

        return (int)pwm;
    }

}
