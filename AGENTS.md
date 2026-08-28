# Agent Guidance

## Map

- `src/Robots`: core library. `Robots.csproj` builds for Rhino3dm/net8, `Robots.Rhino.csproj` for RhinoCommon, and both share `Robots.props`.
- `src/Robots.Grasshopper`: Rhino 8 Grasshopper plug-in and the `Robots.Rhino` developer NuGet package.
- `src/Package.props`: shared product/package metadata used by packable product projects and RhinoPackager. Root `Directory.Build.props` is for repo-wide build defaults only.
- `build/Robots.Build`: release packaging through RhinoPackager.
- `tests/Robots.Tests`: regression tests.

## Safety

- Generated robot code is safety-sensitive. Do not change postprocessor output, target ordering, motion types, zones, speeds, frames, tools, configurations, or formatting unless fixing a confirmed bug or proving equivalence; report possible output changes and run the relevant golden tests.
- Prefer this library's managed implementation over native/Rhino equivalents when both exist, but verify equivalence when behavior matters.

## Code

- Prefer the simplest coherent design. Keep one implementation per behavior and remove unnecessary layers, fallbacks, indirection, defensive branches, and speculative flexibility.
- Fail fast on missing or invalid data and broken invariants. Do not substitute guessed defaults; catch only for expected recovery, cleanup followed by rethrow, or deliberate boundary handling.
- Use `null` only for meaningful absence and validate deserialized configuration immediately. With nullable references enabled, avoid redundant null-only guards; validation that also checks null, such as `ThrowIfNullOrWhiteSpace`, is fine.
- Use current C# when it improves clarity. Omit access modifiers when the default is intended; a member cannot be more visible than its containing type. Use an `Async` suffix only when a synchronous counterpart exists or a framework requires it, and `sealed` only with intent.
- Prefer records and primary constructors for immutable data carriers; use classes for services, mutable state, exceptions, and framework lifecycle types.
- Order C# `using` directives by source: `System`, third-party, `Rhino`, `Grasshopper`, then `Robots`; keep normal imports before aliases/static imports within each group.
- Leave a blank line before an unbraced single-statement `if`, unless it starts the block, and after a multiline statement.
- Prefer arrays for fixed or known-size geometry data. Materialize sequences before counting, indexing, retaining, mutating, or repeated enumeration.
- Treat RhinoCommon geometry, arrays, and domain objects as immutable by convention. Copy only at mutation or ownership boundaries; short-lived owned mutation is fine in hot paths.

## Grasshopper

- New components and parameters should normally derive from the custom `Component` and `Param` base classes.
- Keep component bodies to `RegisterInputParams`, `RegisterOutputParams`, and `SolveComponent`; use helpers and domain types, and keep `GH_*` wrappers in goo, parameter, and data-access infrastructure.
- Domain goo wrappers should inherit the shared `Goo<T, TGoo>` base and add only real custom casting, preview, validation, or serialization behavior.
- Let the shared component base catch exceptions and show runtime messages; component code should throw clear exceptions instead of propagating failure state.
- If code reads an input by visible parameter name, update it when the parameter name changes.
- Obsolete components must keep old GUIDs and port layouts so files deserialize and wires stay attached. Hide them, mark them obsolete, and fail immediately with a clear replacement message.
- Icons should normally be named after the component/parameter class and loaded by convention.

## Build

- Prefer SDK defaults and CLI arguments for one-off variants. Add props or targets only for consumer/runtime contracts, and separate project files only for stable `ProjectReference` targets or outputs that must remain distinct.
- `Robots.Rhino` is compile-time-only for consumers: use `ref/<tfm>/`, keep runtime assemblies out of consumer output, and forward required McNeel props/targets.
- Use `None Pack="true"` for package-only files and `Content` only for real project content. Keep package assets under the owning project's `Resources/Package` folder, with exact `PackagePath`, `TargetPath`, and casing.
- If suppressing a NuGet warning, add a short comment explaining the package contract.

## Verification

- Run the narrowest useful checks, but include `dotnet build` when code changes warrant it.
- For formatting-sensitive edits, run `dotnet format Robots.slnx --verify-no-changes --no-restore --verbosity minimal`.
- For package-resource changes, run `dotnet pack` and inspect `.nupkg` contents.
- For `Robots.Rhino` package changes, verify a disposable `net8.0` consumer builds and its output lacks Robots, RhinoCommon, and Grasshopper runtime assemblies.
- Do not run build and tests in parallel when they write the same output DLLs; build first, then test with `--no-build`.
- When testing same-version local packages, use a workspace-local `NUGET_PACKAGES` folder or clear the relevant cache.
