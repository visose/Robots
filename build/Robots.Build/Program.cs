using RhinoPackager;
using RhinoPackager.Commands;

var app = App.Create(args);
Props props = new("src/Package.props");
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
            project: "src/Robots.Grasshopper/Robots.Grasshopper.csproj",
            target: "build"
        ),
        new Yak
        (
            props: props,
            sourceFolder: "artifacts/bin/Robots.Grasshopper/release",
            tags:
            [
                "rh8_0-any"
            ],
            exclude:
            [
                "*.pdb",
                "*.deps.json"
            ]
        ),
        new Nuget
        (
            props: props,
            project: "src/Robots/Robots.csproj",
            targets: "net8.0"
        ),
        new Nuget
        (
            props: props,
            project: "src/Robots.Grasshopper/Robots.Grasshopper.csproj",
            targets: "net8.0"
        ),
        new Release
        (
            props: props,
            github: github,
            notesFile: "RELEASE",
            message: "> This **release** can only be installed through the package manager in **Rhino 8** using the `_PackageManager` command.\n> Check the [readme](../../blob/master/.github/README.md) for more details."
        )
    ]);

return await app.Run();
