# Duo-PID

### Introduction

The Duo PID project is a device for controlling heating and or cooling operations with PID control. This project is split into two channels each with its own PID and temp sensor. I designed it for use with my fermentaion process when making home brew. However it cold also be used for heating and cooling of spaces in the home or office, other applications that come to mind are seed propergation, 3D Printer enclouser controll and even possibly food dehidrator or anything that required accurate temp control.

**PLEASE NOTE:** *This is a mains powered project, attempt at your own risk.*

### Features

- 2 x Independate PID's
- Independate Tuning functions for each PID
- Adaptive Tuning Mode for each PID
- Independate on/off for each PID
- Heating, Cooling or Both modes
- Settings are saved to EPROM
- Auto Start mode in case of power loss
- Selectable C or F temp readouts

### PCB Gerber Files

Gerber files for PCB manufacture can be found in the Gerber folder.

- Pick the newest revision/version
- Visit https://jlcpcb.com/
- Sign In or Register
- Click 'Order Now'
- Click 'Add Gerber File'
- Select the Gerber .zip file
- Select build options. (I recommend leave defaults, but change qty, colour and select 'Specify a location' under 'Remove Order Number' option. Also make sure both 'SMT Assembly' and 'Stencil' options are off as they are not needed for this.)
- Once your happy with your selection, place order and pay.
- Wait for PCB's to arrive.

### BOM (Build of Materials)

Build of materials can be found the BOM folder. Most of the PCB components can be found in a .csv file, the rest you will need to source yourself from the .txt file. I would recommend places like Amazon, eBay and AliExpress.

- Find the .csv that matches the same revision as the Gerber file.
- Next visit https://lcsc.com/
- Sign In or Register
- Click on the 'BOM Tool' option.
- Now upload the .csv file.
- Set column with C##### numbers to LCSC Part Number.
- Click Next
- Adjust quantities to suit your needs. (Note some parts are optional and other have minimum order quantities)
- Once your happy with your selection, place order and pay.
- Wait for parts to arrive.

### STL Files

STL files are also included in the STL folder, So you can 3D Printer your own enslosure for this project. They are design to fit within the build plate of an Ender 3 (you may need to tweek the default setting to use the whole build plate).

My print/slicer setting for this was as follows;
- 4 top and bottom layers
- 3 layer wall thickness
- 25% infill
- I printed with supports for the box shape areas and support blockers for the rounded areas.
- .4 nozzel
- .2 layer height