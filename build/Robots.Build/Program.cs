using RhinoPackager;
using RhinoPackager.Commands;

var app = App.Create(args);
var github = new Github("visose", "Robots");

app.Add(new ICommand[]
    {
        new Test
        (
            testProject: "tests/Robots.Tests/Robots.Tests.csproj"
        ),
        new CheckVersion(github),
        new Build
        (
            buildProject: "src/Robots.Grasshopper/Robots.Grasshopper.csproj"
        ),
        new Yak
        (
            propsFile: "Directory.Build.props",
            sourceFolder: "artifacts/bin/Robots.Grasshopper/net48",
            files: new []
            {
                "Robots.dll",
                "Robots.gha",
                "ABB.Robotics.Controllers.PC.dll",
                "RobotStudio.Services.RobApi.Desktop.dll",
                "RobotStudio.Services.RobApi.dll"
            },
            tag: "rh7_0-any"
        ),
        new Nuget
        (
            project: "src/Robots/Robots.csproj",
            targets: "netstandard2.0"
        ),
         new Nuget
        (
            project: "src/Robots.Grasshopper/Robots.Grasshopper.csproj",
            targets: "net48"
        ),
        new Release
        (
            github: github,
            file: "RELEASE",
            message: "> This **release** can only be installed through the package manager in **Rhino 7** using the `_PackageManager` command.\n> Check the [readme](../../blob/master/.github/README.md) for more details."
        )
    });

await app.RunAsync();