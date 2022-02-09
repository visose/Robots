<div align="center">
    
# ![Robots](../docs/Images/iconRobot.svg)<br/>robots
### Create and simulate ABB, KUKA, UR and Staubli robot programs

[![License](https://img.shields.io/github/license/visose/Robots?style=flat-square)](../LICENSE)
[![NuGet package](https://img.shields.io/nuget/v/robots?style=flat-square)](https://www.nuget.org/packages/Robots)
[![Repo stars](https://img.shields.io/github/stars/visose/robots?style=flat-square)](../../../)
[![Discussions](https://img.shields.io/github/discussions/visose/robots?style=flat-square)](../../../discussions)
[![Sponsor](https://img.shields.io/badge/sponsor-gray?style=flat-square&logo=GitHub-Sponsors)](https://github.com/sponsors/visose)

**[About](#about) •
[Install](#install) •
[Docs](../../../wiki)**

</div>

## About

**Robots** is a plugin for **[Rhino's](https://www.rhino3d.com/)** **Grasshopper** visual programming interface. It allows users to create and simulate robot programs for **ABB**, **KUKA**, **UR** and **Staubli** robots. It works in **Rhino 7** for **Windows** and **MacOS**.

## Install

> If upgrading from an old version check [here](../../../wiki/home#Upgrading-from-an-older-version).

1. Install in **Rhino 7** using the `_PackageManager` command, search for `Robots`.   
1. Restart **Rhino** and open **Grasshopper**. There should be a new tab in **Grasshopper** named `Robots`.
1. Install a robot library by clicking on the `Libraries` button of a `Load robot system` component.
   > The robots from the library should appear in a **value list** connected to a `Load robot system` component.
1. Read the [docs](../../../wiki) for more info.
1. Check the [samples](../samples/).
   > When opening a sample file, a dialog box might pop up with an assembly not found message. You can close this without fixing the path, it will automatically get fixed after the sample file is loaded.
