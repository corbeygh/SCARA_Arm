

/**
 * Class represents SCARA robotic arm.
 *
 * @Arthur Roberts
 * @0.0
 */

import ecs100.UI;
import java.awt.Color;
import java.util.*;

public class Arm
{

    // fixed arm parameters
    private final int XM1 = 287;  // coordinates of the motor(measured in pixels of the picture)
    private final int YM1 = 374;
    private final int XM2 = 377;
    private final int YM2 = 374;
    private final double R = 154;  // length of the upper/fore arm
    private final double D = XM2-XM1;
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

    /**
     * Constructor for objects of class Arm
     */
    public Arm()
    {
        theta1 = -90.0*Math.PI/180.0; // initial angles of the upper arms
        theta2 = -90.0*Math.PI/180.0;
        valid_state = false;
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
        String out_str=String.format("t1=%3.1f",theta1*180/Math.PI);
        UI.drawString(out_str, XM1-2*mr,YM1-mr/2+2*mr);
        out_str=String.format("XM1=%d",XM1);
        UI.drawString(out_str, XM1-2*mr,YM1-mr/2+3*mr);
        out_str=String.format("YM1=%d",YM1);
        UI.drawString(out_str, XM1-2*mr,YM1-mr/2+4*mr);
        // ditto for second motor
        out_str = String.format("t2=%3.1f",theta2*180/Math.PI);
        UI.drawString(out_str, XM2+2*mr,YM2-mr/2+2*mr);
        out_str=String.format("XM2=%d",XM2);
        UI.drawString(out_str, XM2+2*mr,YM2-mr/2+3*mr);
        out_str=String.format("YM2=%d",YM2);
        UI.drawString(out_str, XM2+2*mr,YM2-mr/2+4*mr);
        // draw Field Of View
        UI.setColor(Color.GRAY);
        UI.drawRect(0,0,640,480);

        // it can b euncommented later when
        // kinematic equations are derived
        if ( valid_state) {
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
        double alpha = (yj1 < yj2) ? Math.atan((yj1-yj2)/Math.abs(xj2-xj1)): Math.atan((yj2-yj1)/Math.abs(xj2-xj1));
        
        // Valid Case
        valid_state = true;
        xt = xa + h * Math.cos((Math.PI/2) - alpha);
        yt = ya - h * Math.sin((Math.PI/2) - alpha);
        xt2 = xa - h*Math.cos(Math.PI/2 - alpha);
        yt2 = ya - h*Math.sin(Math.PI/2 - alpha); 
    }
    
    private void fail(String failMessage){
        valid_state = false;
        UI.clearText();
        UI.println(failMessage);
    }


    //motor angles from tool position
    //updates variables of the class
    public void inverseKinematic(double xt_new,double yt_new){

        valid_state = true;
        xt = xt_new;
        yt = yt_new;
        // distance between pwm and motor
        double d1 = (Math.sqrt(Math.pow(xt-XM1,2) + Math.pow(yt-YM1,2)))/2; //Half distance from t to M1 for trig
        double d2 = (Math.sqrt(Math.pow(XM2-xt,2) + Math.pow(YM2-yt,2)))/2;
        double h1 = Math.sqrt(Math.pow(R,2) - Math.pow(d1,2));
        double h2 = Math.sqrt(Math.pow(R,2) - Math.pow(d2,2));

        double xa1 = XM1 + (xt-XM1)/2;
        double ya1 = YM1 + (yt-YM1)/2;
        double xa2 = xt + (XM2-xt)/2;
        double ya2 = yt + (YM2-yt)/2;

        double alpha1 = Math.atan((yt-YM1)/(xt-XM1));
        double alpha2 = Math.atan((yt-YM2)/(XM2-xt));

        xj1 = xa1 - h1*Math.cos(Math.PI/2-alpha1);
        yj1 = ya1 - h1*Math.sin(Math.PI/2-alpha1);
        xj12 = xa1 + h1*Math.cos(Math.PI/2-alpha1);
        yj12 = ya1 + h1*Math.sin(Math.PI/2-alpha1);

        xj2 = xa2 - h2*Math.cos(Math.PI/2-alpha2);
        yj2 = ya2 - h2*Math.sin(Math.PI/2-alpha2);
        xj22 = xa2 + h2*Math.cos(Math.PI/2-alpha2);
        yj22 = ya2 + h2*Math.sin(Math.PI/2-alpha2);

        if (xj1 == XM1){
            theta1 = 90;
        } else {
            theta1 = (xj1 < XM1) ? 180-Math.toDegrees(Math.asin((yj1-YM1)/R)) : Math.toDegrees(Math.asin((yj1-YM1)/R));
        }
        if (xj2 == XM2){
            theta2 = 90;
        } else {
            theta2 = (xj2 < XM2) ? 180-Math.toDegrees(Math.asin((yj2-YM2)/R)) : Math.toDegrees(Math.asin((yj2-YM2)/R));
        }

        if (d1>2*R){
            valid_state = false;
            return;
        }

        //double l1 = d1/2;
        //double h1 = Math.sqrt(r*r - d1*d1/4);
        // elbows positions
        //xj1 = ...;
        //yj1 = ...;

        ///theta1 = ...;
        if ((theta1>0)||(theta1<-Math.PI)){
            valid_state = false;
            //UI.println("Ange 1 -invalid");
            return;
        }

        // theta12 = atan2(yj12 - YM1,xj12-XM1);
        //double dx2 = xt - XM2;
        //double dy2 = yt - YM2;
        //double d2 =  ;
        //if (d2>2*r){
        // UI.println("Arm 2 - can not reach");
        //valid_state = false;
        //return;
        //}

        //double l2 = d2/2;

        //double h2 = Math.sqrt(r*r - d2*d2/4);
        // elbows positions
        //xj2 = ...;
        //yj2 = ...;
        // motor angles for both 1st elbow positions
        //theta2 = ...;
        //if ((theta2>0)||(theta2<-Math.PI)){
        //valid_state = false;
        //UI.println("Ange 2 -invalid");
        //return;
        //}

        //UI.printf("xt:%3.1f, yt:%3.1f\n",xt,yt);
        //UI.printf("theta1:%3.1f, theta2:%3.1f\n",theta1*180/Math.PI,theta2*180/Math.PI);
        //return;
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
        int pwm = 0;
        return pwm;
    }
    // ditto for motor 2
    public int get_pwm2(){
        int pwm =0;
        //pwm = (int)(pwm2_90 + (theta2 - 90)*pwm2_slope);
        return pwm;
    }

}
