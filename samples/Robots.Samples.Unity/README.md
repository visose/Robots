# Running the Unity project

It requires copying the binaries from Robots (NET Standard 2.0 version) and Rhino3dm.

1. Create a folder inside `Assets` with any name, such as `libs` (it is not required to be `Plugins` anymore)
1. Copy the files listed below. You can find these files in the build folder of the `Robots.Tests` project.
- Robots.dll
- Rhino3dm.dll
- runtimes/win-x64/native/librhino3dm_native.dll (or any other in this folder for other OS platforms)

In the future these files will be distributed via the Unity package manager (hopefully).