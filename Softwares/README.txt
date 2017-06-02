Instructions for the WALL-E 2.0 Workshop

1)	Install WinAVR from "Softwares" Directory

2)	Install Usbasp drivers:
	i)	Plug in the Usbasp Programmer into a USB port of your laptop
	ii)	Open Device Manager, search for "Usbasp" and open it
	iii)	Click "Update" button to Update the driver
	iv)	Choose to select a software already present on the disk (your laptop)
	v)	Select the following directory: ...\WALL-E 2.0\Softwares\usbasp-windriver.2011-05-28

3)	Install Extreme-BurnerAVR from "Softwares" directory

4)	Install Arduino IDE from "Softwares" directory

5)	Copy the folder named "ATmega16 Setup" into "~/Documents/Arduino/hardware".
	Create the "hardware" folder if it doesn't exist. Open Arduino IDE, and a new category in the boards menu (under the "Tools" menu) called "MightyCore" will show up.

6)	Copy the entire "sra16" directory located in "SRA library" directory into "~\Documents\Arduino\libraries"
	Open Arduino IDE, and a new category in the "Include Library.." menu (under the "Edit" menu) called "sra16" will show up.

7)	In the Arduino IDE, under "File" menu, under "Examples" menu, click on the "sra16" option and choose "LedBlink".
	Under the "Boards" menu in the "Tools" menu choose "ATmega16" under "MightyCore" header.
	Under the "Pregrammer" menu in the "Tools" menu choose "Usbasp"
	Compile the code and check for errors. If there are no errors then the setup is complete.
