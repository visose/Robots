# Agent Guidance

## Map

- `src/Robots`: core library. `Robots.csproj` builds the Rhino3dm/net8 package, `Robots.Rhino.csproj` builds the RhinoCommon-backed assembly, and `Robots.props` is shared between both.
- `src/Robots.Grasshopper`: Rhino 8 Grasshopper plug-in and the `Robots.Rhino` developer NuGet package.
- `src/Package.props`: shared product/package metadata used by packable product projects and RhinoPackager. Root `Directory.Build.props` is for repo-wide build defaults only.
- `build/Robots.Build`: release packaging through RhinoPackager.
- `tests/Robots.Tests`: regression tests. Robot-code output tests are safety-critical.

## Safety

- Generated robot code is safety-sensitive. Be conservative with postprocessors and changes to target ordering, motion types, zones, speeds, frames, tools, configurations, or code formatting.
- Do not change postprocessor output unless fixing a confirmed bug or proving behavior equivalence. If output can change, say so and run the relevant golden tests.
- Prefer this library's managed implementation over native/Rhino equivalents when both exist, but verify equivalence when behavior matters.
- Standardize duplicate implementations to one path unless there is a real semantic difference.

## Code

- Fail fast. Throw when required data is missing, invalid, or an invariant is broken.
- Use `null` only for genuinely optional or lifecycle-driven values. Validate deserialized configuration immediately after loading.
- Nullable reference types are enabled; avoid redundant null-only guards on non-nullable parameters. Validation that also happens to check null, such as `ThrowIfNullOrWhiteSpace`, is fine when the non-null validation is useful.
- Prefer current C# syntax when it removes ceremony without hiding intent.
- Order C# `using` directives by source: `System`, third-party, `Rhino`, `Grasshopper`, then `Robots`; keep normal imports before aliases/static imports within each group.
- Leave a blank line above unbraced single-statement `if`s unless they are the first statement in a block.
- Leave a blank line after a multiline statement before starting the next statement.
- Prefer arrays for fixed-size or known-size geometry data: joints, axes, targets, toolpath snapshots, and kinematics internals.
- Avoid `IEnumerable<T>` parameters when data will be counted, indexed, enumerated more than once, or stored. Materialize lazy inputs once at the boundary.
- Treat RhinoCommon geometry, arrays, and project domain objects as immutable by convention, even when their types are technically mutable.
- Avoid defensive copying. Keep references by default, and copy only at the point where code will mutate a value or where an ownership boundary is known to require it.
- Local mutation is fine in hot paths when ownership is clear, the mutable value is short-lived, and it does not leak across API or model boundaries.
- Keep user-facing strings polished: sentence casing, consistent terminology, and no leading spaces.

## Grasshopper

- New components and parameters should normally derive from the custom `Component` and `Param` base classes.
- Component bodies should stay small: override `RegisterInputParams`, `RegisterOutputParams`, and `SolveComponent`; read inputs through helpers, construct domain values, and set outputs.
- Component code should use native/domain types such as `Target`, `IProgram`, `Command`, arrays, and strings. Keep `GH_*` wrappers inside goo, parameter, and data-access infrastructure.
- Domain goo wrappers should inherit the shared `Goo<T, TGoo>` base and add only real custom casting, preview, validation, or serialization behavior.
- Let the shared component base catch exceptions and show runtime messages; component code should throw clear exceptions instead of propagating failure state.
- If code reads an input by visible parameter name, update it when the parameter name changes.
- Obsolete components must keep old GUIDs and port layouts so files deserialize and wires stay attached. Hide them, mark them obsolete, and fail immediately with a clear replacement message.
- Icons should normally be named after the component/parameter class and loaded by convention.

## Build

- Prefer SDK-style project defaults. Add props/targets only for real package or runtime contracts.
- `Robots.Rhino` is compile-time-only for consumers: use `ref/<tfm>/`, keep runtime assemblies out of consumer output, and forward required McNeel props/targets.
- Use `None Pack="true"` for package-only files such as icons, readmes, copied `.props`, and copied `.targets`; avoid `Content` unless the file is real project content.
- Keep project-owned package assets under that project's `Resources/Package` folder.
- Keep `PackagePath`, `TargetPath`, and folder casing exact.
- Prefer CLI arguments for one-off variants. Use separate project files when consumers need stable `ProjectReference` targets or outputs would overwrite each other.
- If suppressing a NuGet warning, add a short comment explaining the package contract.

## Verification

- Run the narrowest useful checks, but include `dotnet build` when code changes warrant it.
- For formatting-sensitive edits, run `dotnet format Robots.slnx --verify-no-changes --no-restore --verbosity minimal`.
- For package-resource changes, run `dotnet pack` and inspect `.nupkg` contents.
- For `Robots.Rhino` package changes, verify a disposable `net8.0` consumer builds and its output lacks Robots, RhinoCommon, and Grasshopper runtime assemblies.
- Do not run build and tests in parallel when they write the same output DLLs; build first, then test with `--no-build`.
- If restore fails because the sandbox cannot read user NuGet config, rerun restore with approved escalation. When testing local packages with the same version, use a workspace-local `NUGET_PACKAGES` folder or clear the relevant cache.
