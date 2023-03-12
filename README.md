# Duo-PID

<img src="https://github.com/idiydownunder/Duo-PID/blob/main/Resources/images/Duo_PID_01.jpg?raw=true" width="600">

### Introduction

The Duo PID project is a device for controlling heating and or cooling operations with PID control. This project is split into two channels each with its own PID and temp sensor. I designed it for use with my fermentaion process when making home brew. However it could also be used for heating and cooling of spaces in the home or office. Other applications that come to mind are seed propergation, 3D Printer enclouser controll and even possibly food dehidrator or anything else that required accurate temp control.

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

### PCB Schematic

<img src="https://github.com/idiydownunder/Duo-PID/blob/main/Resources/images/Duo_PID_v2.1_Schematic.png?raw=true" width="600">

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
- .4mm nozzel
- .2mm layer height
- 4 top and bottom layers
- 3 layer wall thickness
- 25% infill
- I printed with supports for the box shape areas and support blockers for the rounded areas.

### Duo_PID_Code_v1.0.ino

The compiled code is fairly large and as a result requires 29010 bytes of storage, for this reason an Arduino with a Mega 328P chipset is required. The code as makes use of a number of libaries that are required to be installed in your Arduino IDE first. Some are built in libaries that do not require installation, others can be found through the libary manager and others will need to be installed manually.

**Required Libaries** The EEPROM.h and Wire.h libaries are included with the Arduino IDE and should not need to be installed.

The OneWire.h and DallasTemperature.h can be easily installed throught the 'Libary Manager' as shown here;<br>
<img src="https://github.com/idiydownunder/Duo-PID/blob/main/Resources/images/Libary_OneWire.png?raw=true" width="600"><br>
<img src="https://github.com/idiydownunder/Duo-PID/blob/main/Resources/images/Libary_DallasTemperature.png?raw=true" width="600">

The remaining two libaries (ssd1306.h and PID_V1.h) need to be installed manually, the best way to do this is to goto 'Sketch' >> 'Include Libary' >> 'Add .ZIP Libary...'<br>
<img src="https://github.com/idiydownunder/Duo-PID/blob/main/Resources/images/Libary_add_zip.png?raw=true" width="600"><br>
Both of these libary zip's can be found in the Resources/libaries folder within this repository or you can use the links below to view thier GitHub repos if you wish to download them from there;
- [ssd1306.h](https://github.com/lexus2k/ssd1306)
- [PID_v1.h](https://github.com/br3ttb/Arduino-PID-Library)

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons Licence" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.