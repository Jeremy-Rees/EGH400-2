# EGH400-2
Git Repository for Jeremy Rees' (n10214381) EGH400-2 Thesis Project - Synchronised Communications System for Pulsed Power Generator

## Overview
This repository contains three folders - Master, Intermediate and End - which contain all the necessary source code to build each project and install on an STM32F334R8 Nucleo Board via STM32CubeIDE.

## Installation
_Note: These instructions are correct as of STM32CubeIDE version 1.12.0_
1. Click "<> Code" > "Download Zip" to download all files from this repository.
2. Unzip the downloaded file and locate the three project folders - Master, Intermediate and End.
3. Open a new instance of STM32CubeIDE and browse to the desired Workspace folder. Click "Launch".
4. Click "File" > "Import", and select "Projects from Folder or Archive" from the "General" drop down menu. Click "Next".
5. Browse to the desired project folder (Master, Intermediate or End) using the "Directory" button and ensure the project is selected in the import window. Click "Finish".
6. Close the Information Center window by clicking the "x" in the top-left corner of the IDE. This will reveal the imported project.
7. Build the project by pressing "CTRL+B" or by clicking the hammer icon in the toolbar.
8. Load the project onto an STM32F334R8 Nucleo Board connected via USB by clicking the green play button in the toolbar.

The main.c file is located under the project drop down menu > Core > Src > main.c.
