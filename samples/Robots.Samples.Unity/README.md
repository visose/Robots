# Running the Unity project

This is a legacy Unity sample for Unity 2021.2.10f1. It intentionally uses the last published *Robots* NuGet package that targets .NET Standard 2.0, `org.nuget.robots` 1.9.0, through the OpenUPM-hosted UnityNuGet registry.

Do not update this sample to the current source package while *Robots* targets .NET 8. Current Unity releases support managed plug-ins built for .NET Standard, while .NET Core/.NET 8 plug-ins are not supported. Unity has announced CoreCLR and a .NET 10 toolchain for Unity 6.8, but the sample should stay on the pinned .NET Standard package until a compatible Unity version is available and verified.

Assembly Version Validation is disabled in the project settings because UnityNuGet recommends that setting for Unity versions before 2022.2.
