
/* Code for Assignment ??
 * Name:
 * Usercode:
 * ID:
 */

import ecs100.*;
import java.util.*;
import java.io.*;
import java.awt.*;
import javax.swing.SwingUtilities;

/** <description of class Main>
 */
public class Main{

    private Arm arm;
    private Drawing drawing;
    private ToolPath tool_path = new ToolPath();
    // state of the GUI
    private int state; // 0 - nothing
    String host;
    // 1 - inverse point kinematics - point
    // 2 - enter path. Each click adds point
    // 3 - enter path pause. Click does not add the point to the path

    /**      */
    public Main(){
        UI.initialise();
        UI.addButton("xy to angles", this::inverse);
        UI.addButton("Enter path XY", this::enter_path_xy);
        UI.addButton("Save path XY", this::save_xy);
        UI.addButton("Load path XY", this::load_xy);
        UI.addButton("Save path Ang", this::save_ang);
        UI.addButton("Load path Ang:Play", this::load_ang);
        UI.addButton("Load Bulky", this::load_photo);
        UI.addButton("Load Path", this::load_photo_path);
        UI.addButton("Draw", ()->drawing.draw());
        UI.addButton("Send Paths", this::send);


        // UI.addButton("Quit", UI::quit);
        UI.setMouseMotionListener(this::doMouse);
        UI.setKeyListener(this::doKeys);

        //ServerSocket serverSocket = new ServerSocket(22);
        this.arm = new Arm();
        this.drawing = new Drawing();
        this.run();
        arm.draw();
    }

    public void doKeys(String action){
        UI.printf("Key :%s \n", action);
        if (action.equals("b")) {
            // break - stop entering the lines
            state = 3;
            //

        }

    }

    public void doMouse(String action, double x, double y) {
        //UI.printf("Mouse Click:%s, state:%d  x:%3.1f  y:%3.1f\n",
        //   action,state,x,y);
        UI.clearGraphics();
        String out_str=String.format("%3.1f %3.1f",x,y);
        UI.drawString(out_str, x+10,y+10);
        //
        if ((state == 1)&&(action.equals("clicked"))){
            // draw as

            arm.inverseKinematic(x,y);
            arm.draw();
            return;
        }

        if ( ((state == 2)||(state == 3))&&action.equals("moved") ){
            // draw arm and path
            arm.inverseKinematic(x,y);
            arm.draw();

            // draw segment from last entered point to current mouse position
            if ((state == 2)&&(drawing.get_path_size()>0)){
                PointXY lp = new PointXY();
                lp = drawing.get_path_last_point();
                //if (lp.get_pen()){
                UI.setColor(Color.GRAY);
                UI.drawLine(lp.get_x(),lp.get_y(),x,y);
                // }
            }
            drawing.draw();
        }

        // add point
        if ((state == 2) &&(action.equals("clicked"))){
            // add point(pen down) and draw
            UI.printf("Adding point x=%f y=%f\n",x,y);
            drawing.add_point_to_path(x,y,true); // add point with pen down

            arm.inverseKinematic(x,y);
            arm.draw();
            drawing.draw();
            drawing.print_path();
        }

        if ((state == 3) &&(action.equals("clicked"))){
            // add point and draw
            //UI.printf("Adding point x=%f y=%f\n",x,y);
            drawing.add_point_to_path(x,y,false); // add point wit pen up

            arm.inverseKinematic(x,y);
            arm.draw();
            drawing.draw();
            drawing.print_path();
            state = 2;
        }

    }

    public void save_xy(){
        state = 0;
        String fname = UIFileChooser.save();
        drawing.save_path(fname);
    }

    public void enter_path_xy(){
        state = 2;
    }

    public void inverse(){
        state = 1;
        arm.draw();
    }

    public void load_xy(){
        state = 0;
        String fname = UIFileChooser.open();
        drawing.load_path(fname);
        drawing.draw();

        arm.draw();
    }

    // save angles into the file
    public void save_ang(){
        String fname = UIFileChooser.save();
        tool_path.convert_drawing_to_angles(drawing,arm,fname);
        tool_path.convert_angles_to_pwm(arm);
        tool_path.save_pwm_file(fname);
    }

    public void load_ang(){
    }

    public void load_photo(){
        String fname = UIFileChooser.open();
        arm.loadPhoto(fname);
        boolean[][] photo = arm.getPhoto();
        boolean bDraw = false;
        boolean left = true;

        for (int y = 0; y < photo.length; y++){
            //drawing.add_point_to_path(0+Arm.XM1+Arm.D/2-Arm.R/2,0+Arm.YMAX,false);
            if (left){
                for (int x = 0; x < photo[y].length; x++){
                    if (photo[y][x]){
                        if (!bDraw){
                            drawing.add_point_to_path(x+Arm.XM1+Arm.D/2-Arm.R/2,y+Arm.YMAX,false);
                            bDraw = true;
                        }
                        drawing.add_point_to_path(x+Arm.XM1+Arm.D/2-Arm.R/2,y+Arm.YMAX,true);
                    } else {
                        if (bDraw){
                            bDraw = false;
                            drawing.add_point_to_path(x-1+Arm.XM1+Arm.D/2-Arm.R/2,y-1+Arm.YMAX,false);
                        }
                    }
                }
            } else {
                for (int x = photo[y].length-1; x >= 0; x--){
                    if (photo[y][x]){
                        if (!bDraw){
                            drawing.add_point_to_path(x+Arm.XM1+Arm.D/2-Arm.R/2,y+Arm.YMAX,false);
                            bDraw = true;
                        }
                        drawing.add_point_to_path(x+Arm.XM1+Arm.D/2-Arm.R/2,y+Arm.YMAX,true);
                    } else {
                        if (bDraw){
                            bDraw = false;
                            drawing.add_point_to_path(x-1+Arm.XM1+Arm.D/2-Arm.R/2,y-1+Arm.YMAX,false);
                        }
                    }
                }
            }
            left = !left;
        }
    }

    public void load_photo_path(){
        String fname = UIFileChooser.open();
        arm.loadPhoto(fname);
        boolean[][] photo = arm.getPhoto();
        int x = 1;
        int y = 1;
        for (y = 0; y < photo.length; y++){
            for (x = 0; x < photo[y].length; x++){
                if (photo[y][x])break;
            }
            if (x != photo[y].length){
                if (photo[y][x])break;
            }
        }

        while (checkPhoto(photo)){
            boolean start = true;
            while(photo[y][x]){
                if (photo[y][x]){
                    if (start){
                        start = false;
                        drawing.add_point_to_path(x+Arm.XM1+Arm.D/2-Arm.R/2,y+Arm.YMAX,false);
                    }
                    drawing.add_point_to_path(x+Arm.XM1+Arm.D/2-Arm.R/2,y+Arm.YMAX,true);
                }
                photo[y][x] = false;

                if (photo[y][x+1]){x += 1; continue;}
                if (photo[y+1][x]){y += 1; continue;}
                if (photo[y][x-1]){x -= 1; continue;}
                if (photo[y-1][x]){y -= 1; continue;}

                if (photo[y+1][x+1]){x += 1; y += 1; continue;}
                if (photo[y+1][x-1]){x -= 1; y += 1; continue;}
                if (photo[y-1][x+1]){y -= 1; x += 1; continue;}
                if (photo[y-1][x-1]){y -= 1; x -= 1; continue;}
            }
            drawing.add_point_to_path(x+Arm.XM1+Arm.D/2-Arm.R/2,y+Arm.YMAX,false);

            boolean found = false;
            for (int row = 0; row < photo.length; row++){
                for (int col = 0; col < photo[row].length; col++){
                    if (photo[row][col]){
                        x = col;
                        y = row;
                        found = true;
                        break;
                    }
                }
                if (found){
                    found = false;
                    break;
                }
            }
        }
        drawing.add_point_to_path(x+Arm.XM1+Arm.D/2-Arm.R/2,y+Arm.YMAX,false);
    }

    private boolean checkPhoto(boolean[][] photo){
        for (int y = 0; y < photo.length; y++){
            for (int x = 0; x < photo[y].length; x++){
                if (photo[y][x])return true;
            }
        }
        return false;
    }

    //     UI.drawRect(XM1+D/2-R/2,YMAX,R,(YMAX-YMIN)*-1);

    public void run() {
        while(true) {
            arm.draw();
            UI.sleep(20);
        }
    }

    public void send(){
        String fname = UIFileChooser.open();
        if (host == null) host = UI.askString("What is the IP: ");
        try {
            File file = new File(fname);
            Runtime.getRuntime().exec("pscp -l pi -pw pi"+fname+" pi@"+host+":/home/pi/Arm");
        } catch(IOException e){UI.printf("/nImage reading failed: %s/n",e);}

    }
    public static void main(String[] args){
        Main obj = new Main();
    }

}
