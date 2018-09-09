import processing.serial.*;

int prevY_soll = 0;


int prevY_ist = 0;


Serial myPort;
int xpos = 0;
  


void setup() {
  size(1200, 400);
  
prevY_soll = height;


prevY_ist = height;  
  

  println(Serial.list()[6]);

  myPort = new Serial(this, "/dev/tty.usbmodem1411", 57600, 'N', 8, 1.0);

  //myPort.bufferUntil('\n');
  
  drawStuff();
}

void drawStuff() {
  strokeWeight(1);
  stroke(220);
  for (int x=0; x < 12000;x += 100) {
    line(x/10, 0, x/10, height);
  }
  strokeWeight(2);
  for (int x=0; x < 12000;x += 500) {
    line(x/10, 0, x/10, height);
  }
  
  
  strokeWeight(1);
  stroke(220);
  for (int x=0; x < 12000;x += 100) {
    line(x/10, 0, x/10, height);
  }
  strokeWeight(2);
  for (int y=0; y < 120;y += 10) {
    int y_ = (int)map(y, 0, 120.0, 0, height);
    line(0, y_, width, y_);
  }
  
}

void draw() {
  while (myPort.available() > 0) {
    String inBuffer = trim(myPort.readStringUntil('\n'));   
    if (inBuffer != null) {
      //println(">" + inBuffer + "<");
      String[] a = inBuffer.split(";");
      if (a.length == 3) {
        int time = int(a[0]);
        
        float soll = float(a[1]);
        float ist = float(a[2]);
        
        
          println(soll + "    " + ist);
        //println(time + ": " +  soll + "   |   " +  ist);
        
        soll = map(soll, 0, 120.0, 0, height );
        ist = map(ist, 0, 120.0, 0, height);
        
        
        strokeWeight(2);
        
        int x =  (time * 5);
        int x_prev =  ((Math.max(0, time-1)) * 5);
        
        int Y_soll = (int) (height - soll);
        int Y_ist = (int) (height - ist);
        
        
        if (time > 0) {
          stroke(0);
          line(x_prev, prevY_soll, x, Y_soll);
          stroke(204, 102, 0);
          line(x_prev, prevY_ist, x, Y_ist);
        }
        
        prevY_soll = Y_soll;
        prevY_ist = Y_ist;
      }
    }
  }
}

  //void serialEvent (Serial m) {

  //    String inString = null;
  //    while ( (inString = m.readString()) != null) {
  //        inString = trim(inString);
  //        println("+>" + inString + "<+");
  //        inString = inString.substring(0, inString.length() -1).substring(1);
  //        //println("%>" + inString + "<%");
  //        String[] a = inString.split(";");
  //        //if (a.length == 3) {
  //        //  int time = int(a[0]);
  //        //  float soll = float(a[1]);
  //        //  float ist = float(a[2]); 
  //        //  soll = map(soll, 0, 100.0, 0, height);
  //        //  ist = map(ist, 0, 100.0, 0, height);

  //        //  stroke(255);
  //        //  point(time, (int) (height - soll));

  //        //  stroke(135);
  //        //  point(xpos, (int)(height - ist));
  //        //  xpos++;
  //        //} else {
  //        //  println("?");
  //        //}
  //    }

  //}