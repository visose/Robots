<div align="center">
    
# ![Robots](../docs/Images/iconRobot.svg)<br/>robots
### Create and simulate ABB, KUKA, UR and Staubli robot programs

[![License](https://img.shields.io/github/license/visose/Robots?style=flat-square)](../LICENSE)
[![Version](https://img.shields.io/github/v/release/visose/robots?style=flat-square)](../../../releases)
[![Repo stars](https://img.shields.io/github/stars/visose/robots?style=flat-square)](../../../)
[![Discussions](https://img.shields.io/github/discussions/visose/robots?style=flat-square)](../../../discussions)

**[About](#about) •
[Install](#install) •
[Docs](../../../wiki)**

</div>

## About

**Robots** is a plugin for **[Rhino's](https://www.rhino3d.com/)** **Grasshopper** visual programming interface. It allows users to create and simulate robot programs for **ABB**, **KUKA**, **UR** and **Staubli** robots. It works in **Rhino 7** for **Windows** and **MacOS**.

## Install
1. Install in **Rhino 7** using the `_PackageManager` command, search for `Robots`.
   > If you have an older version, delete `Robots.gha` and `Robots.dll` from the `Grasshopper Components` folder.
1. Download at least one pair of `XML` and `3DM` files from the [libraries folder](../libraries). 
   > You can install multiple libraries.
1. Place them inside a folder named `Robots` under the OS's `Documents` folder.
   > In Windows the path will look like `C:\Users\userName\Documents\Robots\RobotLibrary.xml`.<br/>
   > In Mac the path will look like `HD/Users/userName/Robots/RobotLibrary.xml`.
1. Restart **Rhino** and open **Grasshopper**. There should be a new tab in **Grasshopper** named `Robots`.
1. The robots from the library should appear in a **value list** when you place a `Load Robot System` component.
1. Read the [docs](../../../wiki) for more info.
