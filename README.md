# Tiva-Sharp-LCD-Interface
 Tiva Launchpad project for driving the Sharp LCD BoosterPack.
 
 PROJECT DESCRIPTION

This project provides an interface between a Stellaris LM4F120 or Tiva C Series TM4C123G Launchpad and the Sharp LCD
BoosterPack. An example program modified from the grlib_demo.c program provided with TivaWare is included.

The source code is also available from GitHub: https://github.com/DonaldRich/Tiva-Sharp-LCD-Interface.git
PROJECT FEATURES:

The Sharp LCD BoosterPack can be used as an inexpensive and simple display for Stellaris or Tiva LaunchPad projects.
The interface provides an example of the use of the SSI peripheral for SPI communications.
The code was developed using a Stellaris LaunchPad but should run on the Tiva LaunchPad without changes.
Development was done using Code Composer Studio 6.1.0 and TivaWare 2.1.1.71.
Limitations and Warnings:

The capacitive touch sliders on the Sharp LCD BoosterPack are not supported.
Using some functions of the Graphics Library may require increasing the stack size.

RESOURCES:

A Stellaris LM4F120 or Tiva C Series TM4C123G Launchpad and a Sharp LCD BoosterPack are required.

This project uses the TivaWare Driver Library and Graphics Library. Download the libraries from Texas Instruments.

You will also need to use Code Composer Studio to build and install the project.

USE:

Attach the Sharp LCD BoosterPack to the Stellaris or Tiva LaunchPad. Be sure that if you are holding the LaunchPad
board with the USB port way from you, the rocket logo on the BoosterPack is on the left. You may also want to check
that all the pins on the LaunchPad are correctly inserted into the sockets on the BoosterPack.
Unzip the attached zip archive to a folder and import the project into Code Composer Studio.

You may need to change the include and link paths to match the location where you installed the TivaWare libraries.

Build the project and use the Debug command to install it on to an attached Tiva or Stellaris Launchpad. 
