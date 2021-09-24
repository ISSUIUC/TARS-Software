// import libraries
import java.awt.Frame;
import java.awt.BorderLayout;
import controlP5.*; // http://www.sojamo.de/libraries/controlP5/
import processing.serial.*;

/* SETTINGS BEGIN */

// Serial port to connect to
String serialPortName = "/dev/ttyACM0";

// If you want to debug the plotter without using a real serial port set this to true
boolean mockupSerial = false;

/* SETTINGS END */

Serial serialPort; // Serial port object

// interface stuff
ControlP5 cp5;

// Settings for the plotter are saved in this file
JSONObject plotterConfigJSON;

// plots
Graph LineGraph = new Graph(225, 70, 1300, 650, color (255));
float[] barChartValues = new float[9];
float[][] lineGraphValues = new float[9][100];
float[] lineGraphSampleNumbers = new float[100];
color[] graphColors = new color[9];

// helper for saving the executing path
String topSketchPath = "";

void setup() {
  frame.setTitle("Realtime plotter");
  size(1600, 792);

  // set line graph colors
  graphColors[0] = color(131, 255, 20);
  graphColors[1] = color(232, 158, 12);
  graphColors[2] = color(255, 0, 0);
  graphColors[3] = color(62, 12, 232);
  graphColors[4] = color(13, 255, 243);
  graphColors[5] = color(200, 46, 232);
  graphColors[6] = color(62, 12, 232);
  graphColors[7] = color(13, 255, 243);
  graphColors[8] = color(200, 46, 232);

  // settings save file
  topSketchPath = sketchPath();
  plotterConfigJSON = loadJSONObject(topSketchPath+"/plotter_config.json");

  // gui
  cp5 = new ControlP5(this);
  
  // init charts
  setChartSettings();
  // build x axis values for the line graph
  for (int i=0; i<lineGraphValues.length; i++) {
    for (int k=0; k<lineGraphValues[0].length; k++) {
      lineGraphValues[i][k] = 0;
      if (i==0)
        lineGraphSampleNumbers[k] = k;
    }
  }
  
  // start serial communication
  if (!mockupSerial) {
    //String serialPortName = Serial.list()[3];
    serialPort = new Serial(this, serialPortName, 115200);
  }
  else
    serialPort = null;

  // build the gui
  int x = 170;
  int y = 60;
  
  cp5.addTextfield("lgMaxY").setPosition(x, 60).setText(getPlotterConfigString("lgMaxY")).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMinY").setPosition(x, 710).setText(getPlotterConfigString("lgMinY")).setWidth(40).setAutoClear(false);

  fill(255);
  
  cp5.addTextlabel("label").setText("on/off").setPosition(x=13, y=y+170).setColor(255);
  cp5.addTextlabel("multipliers").setText("multipliers").setPosition(x=55, y).setColor(255);
  cp5.addTextfield("lgMultiplier1").setPosition(x=60, y=y+10).setText(getPlotterConfigString("lgMultiplier1")).setColorCaptionLabel(255).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMultiplier2").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier2")).setColorCaptionLabel(255).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMultiplier3").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier3")).setColorCaptionLabel(255).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMultiplier4").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier4")).setColorCaptionLabel(255).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMultiplier5").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier5")).setColorCaptionLabel(255).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMultiplier6").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier6")).setColorCaptionLabel(255).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMultiplier7").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier7")).setColorCaptionLabel(255).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMultiplier8").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier8")).setColorCaptionLabel(255).setWidth(40).setAutoClear(false);
  cp5.addTextfield("lgMultiplier9").setPosition(x, y=y+40).setText(getPlotterConfigString("lgMultiplier9")).setColorCaptionLabel(255).setWidth(40).setAutoClear(false);
  cp5.addToggle("lgVisible1").setPosition(x=x-50, y=240).setValue(int(getPlotterConfigString("lgVisible1"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[0]);
  cp5.addToggle("lgVisible2").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible2"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[1]);
  cp5.addToggle("lgVisible3").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible3"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[2]);
  cp5.addToggle("lgVisible4").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible4"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[3]);
  cp5.addToggle("lgVisible5").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible5"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[4]);
  cp5.addToggle("lgVisible6").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible6"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[5]);
  cp5.addToggle("lgVisible7").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible7"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[6]);
  cp5.addToggle("lgVisible8").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible8"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[7]);
  cp5.addToggle("lgVisible9").setPosition(x, y=y+40).setValue(int(getPlotterConfigString("lgVisible9"))).setMode(ControlP5.SWITCH).setColorActive(graphColors[8]);
}

byte[] inBuffer = new byte[100]; // holds serial message
int i = 0; // loop variable
void draw() {
  /* Read serial and update values */
  if (mockupSerial || serialPort.available() > 0) {
    String myString = "";
    if (!mockupSerial) {
      try {
        serialPort.readBytesUntil('\r', inBuffer);
      }
      catch (Exception e) {
      }
      myString = new String(inBuffer);
    }
    else {
      myString = mockupSerialFunction();
    }

    //println(myString);

    // split the string at delimiter (space)
    String[] nums = split(myString, ' ');
    
    int numberOfInvisibleLineGraphs = 0;
    for (i=0; i<9; i++) {
      if (int(getPlotterConfigString("lgVisible"+(i+1))) == 0) {
        numberOfInvisibleLineGraphs++;
      }
    }

    // build the arrays for bar charts and line graphs
    for (i=0; i<nums.length; i++) {

      // update line graph
      try {
        if (i<lineGraphValues.length) {
          for (int k=0; k<lineGraphValues[i].length-1; k++) {
            lineGraphValues[i][k] = lineGraphValues[i][k+1];
          }

          lineGraphValues[i][lineGraphValues[i].length-1] = float(nums[i])*float(getPlotterConfigString("lgMultiplier"+(i+1)));
        }
      }
      catch (Exception e) {
      }
    }
  }

  // draw the bar chart
  background(50);

  // draw the line graphs
  LineGraph.DrawAxis();
  for (int i=0;i<lineGraphValues.length; i++) {
    LineGraph.GraphColor = graphColors[i];
    if (int(getPlotterConfigString("lgVisible"+(i+1))) == 1)
      LineGraph.LineGraph(lineGraphSampleNumbers, lineGraphValues[i]);
  }
}

// called each time the chart settings are changed by the user 
void setChartSettings() {
  LineGraph.xLabel=" Samples ";
  LineGraph.yLabel="Value";
  LineGraph.Title="";  
  LineGraph.xDiv=20;  
  LineGraph.xMax=0; 
  LineGraph.xMin=-100;  
  LineGraph.yMax=int(getPlotterConfigString("lgMaxY")); 
  LineGraph.yMin=int(getPlotterConfigString("lgMinY"));
}

// handle gui actions
void controlEvent(ControlEvent theEvent) {
  if (theEvent.isAssignableFrom(Textfield.class) || theEvent.isAssignableFrom(Toggle.class) || theEvent.isAssignableFrom(Button.class)) {
    String parameter = theEvent.getName();
    String value = "";
    if (theEvent.isAssignableFrom(Textfield.class))
      value = theEvent.getStringValue();
    else if (theEvent.isAssignableFrom(Toggle.class) || theEvent.isAssignableFrom(Button.class))
      value = theEvent.getValue()+"";

    plotterConfigJSON.setString(parameter, value);
    saveJSONObject(plotterConfigJSON, topSketchPath+"/plotter_config.json");
  }
  setChartSettings();
}

// get gui settings from settings file
String getPlotterConfigString(String id) {
  String r = "";
  try {
    r = plotterConfigJSON.getString(id);
  } 
  catch (Exception e) {
    r = "";
  }
  return r;
}
