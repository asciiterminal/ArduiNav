/*
 ACOUSTIC RADAR RECEIVER 1
 acoustic_radar_receiver_1.pde
 Copyright (C) 2018 lingib
 Last update 17 June 2018
 
 This program generates a "radar-type" display om which the distance(s) versus 
 azimuth from an ultrasonic sensor are plotted.
 
 The code for the fadeGraphics() routine was found at
 https://forum.processing.org/two/discussion/13189/a-better-way-to-fade
 
 All other code was created by lingib
 https://www.instructables.com/member/lingib/instructables/
 
 ----------
 Copyright
 ----------
 This is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License. If 
 not, see <http://www.gnu.org/licenses/>.
 */

// ======================
// globals
// ======================

// ----- serial port
import processing.serial.*;             //import the serial library
Serial myPort;                          //the Serial port object
final int Baud_rate = 115200;           //communication speed
String Input_string;                    //for incoming data 
boolean Connected = false;              //flag          

// ----- display graphics
PGraphics Canvas;                       //name of drawing area to be created
PFont myFont;                           //name of font to be created

// ----- ultrasonic beam
int Azimuth = 0;                        //radians (180 degrees equals PI radians)
int [][] Ping = new int [257][2];       //257 rows of 2 columns
int Direction = 0;                      //scan direction: true=CW, false=CCW

// ======================
// setup
// ======================
void setup() 
{
  // ----- image window
  //size(900, 600, P3D);                                       //P3D parameter allows rotation around Z-axis
  size(1200, 800, P3D);                                       //P3D parameter allows rotation around Z-axis

  // ----- create a drawing area for fading the beam
  Canvas = createGraphics(width, height);                          

  // ------ create the screen font
  myFont = createFont("Arial Black", 20);

  // ----- initialize the serial port
  printArray(Serial.list());                                //lists your COM ports on screen
  myPort = new Serial(this, Serial.list()[0], Baud_rate);
  myPort.bufferUntil('\n');
}

// ======================
// draw
// ======================
void draw() 

{
  // ----- define colors, scale, & text
  background(0);                                    //black background
  textFont(myFont, 20);                            //specify font to be used

  // ----- draw beam on its own canvas
  Canvas.beginDraw();
  Canvas.translate(width/2, height*0.8);            //beam origin
  Canvas.stroke(0, 255, 0);                         //green beam
  Canvas.strokeWeight(7);                           //set beam-width
  Canvas.scale(0.8);                                //think 100% but scale 80%
  Canvas.rotate(-Azimuth*PI/256);                   //rotate "sheet of paper" but
  Canvas.line(0, 0, width/2, 0);                    //think horizontal lines  
  Canvas.endDraw();

  // ----- draw the graticule
  draw_graticule();

  // ----- plot CCW data
  if (Direction == 0) 
  {
    for (int i=0; i<Azimuth+1; i++)
    {
      plot_data(i);                                    //plot data points BELOW azimuth

    }
  }

  // ----- plot CW data
  if (Direction == 1) 
  {
    for (int i=Azimuth; i<257; i++)
    {
      plot_data(i);                                   //plot data points ABOVE azimuth
    }
  }

  // ----- superimpose beam over the display canvas
  image(Canvas, 0, 0);  

  // ----- fade the beam
  fadeGraphics(Canvas, 5);                           //the number controls the beam width
}

// =======================
// serial event  (called with each Arduino data string)
// =======================
void serialEvent(Serial myPort)
{
  // ----- wait for a line-feed
  Input_string = myPort.readStringUntil('\n');

  // ----- validate
  if (Input_string != null) 
  {
    // ----- trim whitespace
    Input_string = trim(Input_string);
    println(Input_string);

    // ----- make contact
    if (Connected == false) 
    {
      if (Input_string.equals("S")) 
      {
        // ----- set flag
        Connected = true;        //connection made

        // ----- request data
        myPort.clear();            //clear the receive buffer
        myPort.write("S");         //request data
      }
    } else 
    // ----- send data
    {
      Input_string = trim(Input_string);              //remove leading/trailing whitespace 
      println(Input_string);

      int[] values = int(split(Input_string, ','));
      Azimuth = values[0];
      Ping[Azimuth][0] = values[1];
      Ping[Azimuth][1] = values[2];
      Direction = values[3];

      println(Azimuth);
      println(Ping[Azimuth][0]);
      println(Ping[Azimuth][1]);
      println(Direction);

      myPort.clear();                                //clear the receive buffer
      myPort.write("S");
    }
  }
}

// =======================
// draw graticule (horizontal text)
// =======================
void draw_graticule()
{
  // ----- setup
  pushMatrix();                                     //save screen parameters
  translate(width/2, height*0.8);                   //move the origin
  scale(0.8);                                       //scale everything 80%

  // ----- draw the arcs
  stroke(128);                                      //use gray lines
  arc(0, 0, width, -width, 0, PI, CHORD);           //CHORD draws the baseline
  arc(0, 0, width*0.75, -width*0.75, 0, PI, OPEN);
  arc(0, 0, width*0.5, -width*0.5, 0, PI, OPEN);
  arc(0, 0, width*0.25, -width*0.25, 0, PI, OPEN);

  // ----- draw the radials
  pushMatrix();                                     //save screen parameters
  stroke(128);                                      //use gray lines
  rotateZ(-radians(45));                            //rotate the screen coordinates
  line(0, 0, width/2, 0);                           //draw line at 45  
  rotateZ(-radians(45));                            //rotate another 45 degrees
  line(0, 0, width/2, 0);                           //draw line at 90  
  rotateZ(-radians(45));                            //rotate another 45 degrees
  line(0, 0, width/2, 0);                           //draw line at 135
  popMatrix();                                      //restore screen parameters

  // ----- label the radials
  fill(0, 0, 255);                                  //blue text
  textAlign(LEFT, CENTER);
  text("0", width/2+5, 0);                          //"0" degrees

  textAlign(LEFT, BOTTOM);
  text("45", width*0.35+5, -width*0.35);             //"45" degrees

  textAlign(RIGHT, BOTTOM);
  text("90", -5, -width/2);                         //"90" degrees

  textAlign(RIGHT, BOTTOM);
  text("135", -width*0.35-5, -width*0.35);          //"135" degrees

  textAlign(RIGHT, CENTER);
  text("180", -width/2-5, 0);                       //"180" degrees

  // ----- label the arcs
  fill(255);                                        //light gray text
  textAlign(LEFT, BOTTOM);
  text("100cm", +5, -width/2);                      //"100cm"
  text("75", +5, -width/2*0.75);                    //"75cm"
  text("50", +5, -width/2*0.5);                     //"50cm"
  text("25", +5, -width/2*0.25);                    //"25cm"

  // ----- restore properties
  strokeWeight(1); 
  fill(0);                                          //white 
  stroke(255);                                      //black
  scale(1.0);
  popMatrix();                                      //restore screen parameters
}

// =======================
// plot data
// =======================
void plot_data(int index)
{
  // ----- setup
  pushMatrix();                          //save screen parameters
  translate(width/2, height*0.8);        //move the origin  
  scale(0.8);

  // ----- plot array contents
  rotateZ(-index*PI/256);                //rotate the display coordinates
  strokeWeight(5);                       //set data size

  stroke(255, 0, 0);                     //set data1 color to red
  if (Ping[index][0]>100) Ping[index][0] = 1000;      //hide by printing off-screen 
  ellipse(width/2*Ping[index][0]/100, 0, 5, 5);      //plot data1

  stroke(0, 0, 255);                     //set data2 color to blue           
  if (Ping[index][1]>100) Ping[index][1] = 1000;      //hide by printing off-screen 
  ellipse(width/2*Ping[index][1]/100, 0, 5, 5);      //plot data2

  // ----- restore defaults
  strokeWeight(1);
  stroke(0);
  popMatrix();                           //restore screen parameters
} 

// =======================
// fadeGraphics
// =======================
/*
   This fadeGraphics() routine was found at
 https://forum.processing.org/two/discussion/13189/a-better-way-to-fade
 */
void fadeGraphics(PGraphics c, int fadeAmount) 
{
  c.beginDraw();
  c.loadPixels();

  // ----- iterate over pixels
  for (int i =0; i<c.pixels.length; i++) 
  { 
    // ----- get alpha value
    int alpha = (c.pixels[i] >> 24) & 0xFF ;

    // ----- reduce alpha value
    alpha = max(0, alpha-fadeAmount);

    // ----- assign color with new alpha-value
    c.pixels[i] = alpha<<24 | (c.pixels[i]) & 0xFFFFFF ;
  }

  Canvas.updatePixels();
  Canvas.endDraw();
}
