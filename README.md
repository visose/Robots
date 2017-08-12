
![Home](https://github.com/visose/Robots/blob/master/Documentation/Images/title.png)

Grasshopper plugin for programming ABB, KUKA and UR robots

* Download the binary from the [latest release](https://github.com/visose/Robots/releases).
* Download a [robot library](https://github.com/visose/Robots/wiki/Robot-libraries).
* Download the [example files](https://github.com/visose/Robots/tree/master/Documentation/Examples) (they work with the Bartlett robot library).
* Read the [Wiki](https://github.com/visose/Robots/wiki).

#### Changes
The following has been changed the [original version](https://github.com/visose/Robots/releases)

* ABB file split was changed to use startLoad and waitLoad instead of Load and UnLoad. This reduces pause time between module changes. However you will essentially have 2 modules loaded at once, so you will need to adjust sizes as required.
