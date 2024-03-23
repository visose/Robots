using RhinoPackager;
using RhinoPackager.Commands;

var app = App.Create(args);
Props props = new("Directory.Build.props");
Github github = new("visose", "Robots");

app.Add(
    [
        new Test
        (
            testProject: "tests/Robots.Tests/Robots.Tests.csproj"
        ),
        new CheckVersion
        (
            props: props,
            github: github
        ),
        new Build
        (
            buildProject: "src/Robots.Grasshopper/Robots.Grasshopper.csproj"
        ),
        new Yak
        (
            props: props,
            sourceFolder: "artifacts/bin/Robots.Grasshopper/release",
            files:
            [
                "Robots.dll",
                "Robots.gha",
                "ABB.Robotics.Controllers.PC.dll",
                "RobotStudio.Services.RobApi.Desktop.dll",
                "RobotStudio.Services.RobApi.dll",
                "Renci.SshNet.dll",
                "MathNet.Numerics.dll",
                "Kdl.NetStandard.dll",
                "kdl_wrap.dll",
                "iconRobot.png"
            ],
            tags:
            [
                "rh7_0-any",
                "rh8_0-any"
            ]
        ),
        new Nuget
        (
            props: props,
            project: "src/Robots/Robots.csproj",
            targets: "netstandard2.0"
        ),
         new Nuget
        (
            props: props,
            project: "src/Robots.Grasshopper/Robots.Grasshopper.csproj",
            targets: "net48"
        ),
        new Release
        (
            props: props,
            github: github,
            notesFile: "RELEASE",
            message: "> This **release** can only be installed through the package manager in **Rhino 7** or **Rhino 8** using the `_PackageManager` command.\n> Check the [readme](../../blob/master/.github/README.md) for more details."
        )
    ]);

await app.RunAsync();
