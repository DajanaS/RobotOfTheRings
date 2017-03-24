package makecoordinates;

import java.io.*;
import java.util.ArrayList;

class Point {

    double x;
    double y;
    double z;
    double w;

    Point(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

}

public class MakeCoordinates {

    public static void main(String[] args) throws FileNotFoundException, IOException {

        //File file = new File("reachable_goals.txt");
        FileInputStream fstream = new FileInputStream("reachable_goals.txt");
        ArrayList<Point> coordinates_array = new ArrayList<>();

        BufferedReader br = new BufferedReader(new InputStreamReader(fstream));

        String strLine;
        int lineCounter = 1;
        String[] line;
        Point p;
        double x = 0, y = 0, z = 0, w = 0;

//Read File Line By Line
        while ((strLine = br.readLine()) != null) {

            if (lineCounter == 17) {
                lineCounter = 0;
                p = new Point(x, y, z, w);
                coordinates_array.add(p);
                System.out.println("x: " + x + " y: " + y + " z: " + z + " w: " + w);
            }

            if (lineCounter == 9 || lineCounter == 10 || lineCounter == 15 || lineCounter == 16) {
                line = strLine.split(" ");
                switch (lineCounter) {
                    case 9:
                        x = Double.parseDouble(line[line.length - 1]);
                        break;
                    case 10:
                        y = Double.parseDouble(line[line.length - 1]);
                        break;
                    case 15:
                        z = Double.parseDouble(line[line.length - 1]);
                        break;
                    case 16:
                        w = Double.parseDouble(line[line.length - 1]);
                        break;
                }
            }
            lineCounter++;
        }

        br.close();

        createGoals(coordinates_array);
    }

    public static ArrayList<Point> createGoals(ArrayList<Point> arr) {

        return arr;
    }
}
