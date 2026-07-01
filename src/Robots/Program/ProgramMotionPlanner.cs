using static System.Math;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

class ProgramMotionPlanner
{
    const double ContinuationStep = 50.0;

    readonly record struct TargetSpeed(double DeltaTime, double MinTime, int LeadingJoint, SpeedType Type);
    readonly record struct SegmentSpeed(double DeltaTime, double MinTime);
    readonly record struct JointSpeed(double Time, int LeadingJoint);
    readonly record struct MotionWarn(int Index, int Group, int Joint);
    readonly record struct ZoneCap(int Index, int Group, string Side, double Requested, double Used);
    enum EndpointMode { Interpolated, Direct, Continuation }

    readonly Program _program;
    readonly RobotSystem _robotSystem;
    int _stationaryCount;
    int _rotationCount;
    int _axisCount;
    int _externalCount;
    int _zoneCapCount;
    int _lastStationary = -1;
    int _lastRotation = -1;
    int _lastAxis = -1;
    int _lastExternal = -1;
    MotionWarn _firstStationary;
    MotionWarn _firstRotation;
    MotionWarn _firstAxis;
    MotionWarn _firstExternal;
    ZoneCap _firstZoneCap;
    ZoneCap _worstZoneCap;

    internal List<SystemTarget> Keyframes { get; } = [];
    internal List<SystemTarget> FixedTargets { get; }

    internal ProgramMotionPlanner(Program program, List<SystemTarget> systemTarget, double stepSize)
    {
        _ = CheckFinite(stepSize, nameof(stepSize), "Step size must be finite.");
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(stepSize);

        _robotSystem = program.RobotSystem;
        _program = program;

        FixFirstTarget(systemTarget[0]);

        var indexError = _program.Errors.Count == 0
            ? FixTargetMotions(systemTarget, stepSize)
            : 0;

        AddMotionWarnings();

        if (indexError == -1 && _program.Errors.Count == 0)
        {
            ApplyFlybyKeyframes(systemTarget, stepSize);
            AddZoneCapWarnings();
        }

        FixedTargets = (indexError != -1)
             ? systemTarget.GetRange(0, indexError + 1)
             : systemTarget;
    }

    void FixFirstTarget(SystemTarget firstTarget)
    {
        if (firstTarget.ProgramTargets.All(x => x.IsJointTarget))
            return;

        var targets = firstTarget.ProgramTargets.Map(x => x.Target);
        var kinematics = _robotSystem.Kinematics(targets);

        foreach (var programTarget in firstTarget.ProgramTargets)
        {
            if (programTarget.IsJointTarget)
                continue;

            var kinematic = kinematics[programTarget.Group];

            if (kinematic.Errors.Count > 0)
            {
                _program.AddError(IssueKind.KinematicError, $"Errors in target {programTarget.Index} of robot {programTarget.Group}:", programTarget.Index, programTarget.Group, nameof(ProgramMotionPlanner));

                foreach (var error in kinematic.Errors)
                    _program.AddError(IssueKind.KinematicError, error, programTarget.Index, programTarget.Group, nameof(ProgramMotionPlanner));
            }

            int robotJointCount = _robotSystem.GetRobotJointCount(programTarget.Group);
            programTarget.Target = new JointTarget(kinematic.Joints[..robotJointCount], programTarget.Target);

            if (_robotSystem.RequiresContinuation(programTarget.Group))
            {
                _program.AddError(IssueKind.KinematicError, $"First target in robot {programTarget.Group} should be a joint target because this robot needs a known starting joint state.", programTarget.Index, programTarget.Group, nameof(ProgramMotionPlanner));
            }
            else
            {
                _program.AddWarning(IssueKind.KinematicError, $"First target in robot {programTarget.Group} changed to a joint target.", programTarget.Index, programTarget.Group, nameof(ProgramMotionPlanner));
            }
        }
    }

    int FixTargetMotions(List<SystemTarget> systemTargets, double stepSize)
    {
        double time = 0;
        double[][]? prevJoints = null;

        for (int i = 0; i < systemTargets.Count; i++)
        {
            var systemTarget = systemTargets[i];
            prevJoints = i == 0
                ? SetFirstKinematics(systemTarget, systemTargets)
                : CheckMotionSegment(systemTargets, i, prevJoints, stepSize, ref time);

            if (_program.Errors.Count > 0)
            {
                _program.Duration = time;
                return i;
            }
        }

        _program.Duration = time;
        return -1;
    }

    double[][] SetFirstKinematics(SystemTarget systemTarget, List<SystemTarget> systemTargets)
    {
        var targets = systemTarget.ProgramTargets.Map(x => x.Target);
        var kinematics = _robotSystem.Kinematics(targets);
        systemTarget.SetTargetKinematics(kinematics, _program);
        CheckUndefined(systemTarget, systemTargets);
        Keyframes.Add(systemTarget.ShallowClone());

        return kinematics.Map(k => k.Joints);
    }

    double[][]? CheckMotionSegment(List<SystemTarget> systemTargets, int index, double[][]? prevJoints, double stepSize, ref double time)
    {
        var systemTarget = systemTargets[index];
        var previous = systemTargets[index - 1];
        var modes = systemTarget.ProgramTargets.Map(GetEndpointMode);

        if (modes.Any(mode => mode != EndpointMode.Interpolated))
            prevJoints = SolveEndpoints(systemTarget, previous, prevJoints, modes);

        if (_program.Errors.Count > 0)
            return prevJoints;

        var prevStep = InterpolateSegment(systemTarget, previous, modes, ref prevJoints, stepSize, ref time, out var segmentTime);

        if (_program.Errors.Count > 0)
            return prevJoints;

        AddWaitTime(systemTarget, prevStep, ref time);
        SetInterpolatedEndpoint(systemTarget, prevStep, modes);
        CheckUndefined(systemTarget, systemTargets);
        FinalizeTarget(systemTarget, prevStep, time, segmentTime);

        return prevJoints;
    }

    SystemTarget InterpolateSegment(SystemTarget systemTarget, SystemTarget previous, IReadOnlyList<EndpointMode> modes, ref double[][]? prevJoints, double stepSize, ref double time, out SegmentSpeed totalSpeed)
    {
        int divisions = GetDivisions(systemTarget, previous, stepSize);
        var prevStep = ResetTiming(previous.ShallowClone());

        double lastDeltaTime = 0;
        double deltaSinceKey = 0;
        double minSinceKey = 0;
        totalSpeed = default;

        for (int step = 1; step <= divisions; step++)
        {
            double t = step / (double)divisions;
            var interTarget = systemTarget.ShallowClone();
            var targets = systemTarget.Lerp(previous, _robotSystem, t, 0.0, 1.0);
            var kinematics = _program.RobotSystem.Kinematics(targets, prevJoints);

            prevJoints = kinematics.Map(k => k.Joints);
            interTarget.SetTargetKinematics(kinematics, _program, prevStep);

            var speed = GetSegmentSpeed(interTarget, prevStep, 1.0 / divisions);

            if ((step > 1) && ShouldStartNewKeyframe(modes, speed.DeltaTime, lastDeltaTime))
            {
                Keyframes.Add(prevStep.ShallowClone());
                deltaSinceKey = 0;
                minSinceKey = 0;
            }

            lastDeltaTime = speed.DeltaTime;
            time += speed.DeltaTime;
            totalSpeed = new(totalSpeed.DeltaTime + speed.DeltaTime, totalSpeed.MinTime + speed.MinTime);
            deltaSinceKey += speed.DeltaTime;
            minSinceKey += speed.MinTime;

            interTarget.DeltaTime = deltaSinceKey;
            interTarget.MinTime = minSinceKey;
            interTarget.TotalTime = time;

            prevStep = interTarget;

            if (_program.Errors.Count > 0)
                break;
        }

        Keyframes.Add(prevStep.ShallowClone());
        return prevStep;
    }

    static int GetDivisions(SystemTarget systemTarget, SystemTarget previous, double stepSize)
    {
        int divisions = 1;

        foreach (var target in systemTarget.ProgramTargets.Where(x => !x.IsJointMotion))
        {
            var prevTarget = previous.ProgramTargets[target.Group];
            var prevPlane = target.GetPrevPlane(prevTarget);
            double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
            divisions = Max(divisions, (int)Ceiling(distance / stepSize));
        }

        return divisions;
    }

    void ApplyFlybyKeyframes(List<SystemTarget> systemTargets, double stepSize)
    {
        if (!HasFlyby(systemTargets))
            return;

        List<SystemTarget> keyframes = [];
        var first = ResetTiming(systemTargets[0].ShallowClone());
        keyframes.Add(first);

        var prevJoints = first.ProgramTargets.Map(x => x.Kinematics.Joints);

        for (int i = 1; i < systemTargets.Count; i++)
        {
            var previous = systemTargets[i - 1];
            var current = systemTargets[i];
            var entryFractions = GetEntryFractions(systemTargets, i);
            double entryTime = SegmentTime(previous.TotalTime, current.TotalTime, entryFractions.Min());
            var entry = CreateFractionKeyframe(previous, current, entryFractions, entryTime, ref prevJoints, keyframes[^1]);
            AddKeyframe(keyframes, entry);

            if (_program.Errors.Count > 0)
                return;

            if (!HasFlyby(systemTargets, i))
                continue;

            var next = systemTargets[i + 1];
            var exitFractions = GetExitFractions(systemTargets, i);
            double exitTime = SegmentTime(current.TotalTime, next.TotalTime, exitFractions.Max());
            var exitPreviewJoints = entry.ProgramTargets.Map(x => x.Kinematics.Joints);
            var exitPreview = CreateFractionKeyframe(current, next, exitFractions, exitTime, ref exitPreviewJoints, entry);

            if (_program.Errors.Count > 0)
                return;

            int divisions = GetFlybyDivisions(current, entry, exitPreview, stepSize);

            for (int step = 1; step < divisions; step++)
            {
                double t = step / (double)divisions;
                double time = SegmentTime(entry.TotalTime, exitTime, t);
                var blend = CreateFlybyKeyframe(current, next, entry, exitPreview, t, time, ref prevJoints, keyframes[^1]);
                AddKeyframe(keyframes, blend);

                if (_program.Errors.Count > 0)
                    return;
            }

            var exit = CreateFractionKeyframe(current, next, exitFractions, exitTime, ref prevJoints, keyframes[^1]);
            AddKeyframe(keyframes, exit);

            if (_program.Errors.Count > 0)
                return;
        }

        Keyframes.Clear();
        Keyframes.AddRange(keyframes);
    }

    static bool HasFlyby(List<SystemTarget> systemTargets)
    {
        for (int i = 1; i < systemTargets.Count - 1; i++)
        {
            if (HasFlyby(systemTargets, i))
                return true;
        }

        return false;
    }

    static bool HasFlyby(List<SystemTarget> systemTargets, int index) =>
        index > 0
        && index < systemTargets.Count - 1
        && systemTargets[index].ProgramTargets.Any(IsFlyby);

    static bool HasFlyby(List<SystemTarget> systemTargets, int index, int group) =>
        index > 0
        && index < systemTargets.Count - 1
        && IsFlyby(systemTargets[index].ProgramTargets[group]);

    static bool IsFlyby(ProgramTarget target) =>
        target.Target.Zone.IsFlyBy;

    double[] GetEntryFractions(List<SystemTarget> systemTargets, int index)
    {
        var previous = systemTargets[index - 1];
        var current = systemTargets[index];
        var fractions = new double[current.ProgramTargets.Count];

        for (int group = 0; group < fractions.Length; group++)
        {
            var target = current.ProgramTargets[group];
            fractions[group] = HasFlyby(systemTargets, index, group)
                ? 1.0 - GetZoneFraction(previous.ProgramTargets[group], target, target, "incoming")
                : 1.0;
        }

        return fractions;
    }

    double[] GetExitFractions(List<SystemTarget> systemTargets, int index)
    {
        var current = systemTargets[index];
        var next = systemTargets[index + 1];
        var fractions = new double[current.ProgramTargets.Count];

        for (int group = 0; group < fractions.Length; group++)
        {
            var target = current.ProgramTargets[group];
            fractions[group] = HasFlyby(systemTargets, index, group)
                ? GetZoneFraction(target, next.ProgramTargets[group], target, "outgoing")
                : 0.0;
        }

        return fractions;
    }

    double GetZoneFraction(ProgramTarget from, ProgramTarget to, ProgramTarget corner, string side)
    {
        double distance = GetPathDistance(from, to, out bool isCartesianDistance);

        if (distance <= DistanceTol)
            return 0.0;

        var zone = corner.Target.Zone;
        double maxDistance = 0.5 * distance;

        if (isCartesianDistance && zone.Distance > maxDistance + DistanceTol)
            TrackZoneCap(corner, side, zone.Distance, maxDistance);

        return Min(0.5, zone.Distance / distance);
    }

    static double GetPathDistance(ProgramTarget from, ProgramTarget to, out bool isCartesianDistance)
    {
        double distance = from.WorldPlane.Origin.DistanceTo(to.WorldPlane.Origin);

        if (distance > DistanceTol)
        {
            isCartesianDistance = true;
            return distance;
        }

        isCartesianDistance = false;

        var fromJoints = from.Kinematics.Joints;
        var toJoints = to.Kinematics.Joints;
        double sum = 0;

        for (int i = 0; i < fromJoints.Length; i++)
        {
            double delta = toJoints[i] - fromJoints[i];
            sum += delta * delta;
        }

        return Sqrt(sum);
    }

    static double SegmentTime(double start, double end, double t) =>
        start + (end - start) * Clamp(t, 0.0, 1.0);

    SystemTarget CreateFractionKeyframe(SystemTarget previous, SystemTarget current, IReadOnlyList<double> fractions, double time, ref double[][] prevJoints, SystemTarget previousKeyframe)
    {
        var keyframe = current.ShallowClone();
        var targets = current.LerpFractions(previous, _robotSystem, fractions);
        var kinematics = _robotSystem.Kinematics(targets, prevJoints);
        prevJoints = kinematics.Map(k => k.Joints);
        keyframe.SetTargetKinematics(kinematics, _program, previousKeyframe);
        SetKeyframeTiming(keyframe, time, previousKeyframe);
        return keyframe;
    }

    SystemTarget CreateFlybyKeyframe(SystemTarget corner, SystemTarget next, SystemTarget entry, SystemTarget exit, double t, double time, ref double[][] prevJoints, SystemTarget previousKeyframe)
    {
        var keyframe = corner.ShallowClone();
        var targets = new Target[corner.ProgramTargets.Count];

        for (int group = 0; group < targets.Length; group++)
        {
            var cornerTarget = corner.ProgramTargets[group];
            targets[group] = IsFlyby(cornerTarget)
                ? CreateFlybyTarget(cornerTarget, next.ProgramTargets[group], entry.ProgramTargets[group], exit.ProgramTargets[group], t)
                : cornerTarget.Target;
        }

        var kinematics = _robotSystem.Kinematics(targets, prevJoints);
        prevJoints = kinematics.Map(k => k.Joints);
        keyframe.SetTargetKinematics(kinematics, _program, previousKeyframe);
        SetKeyframeTiming(keyframe, time, previousKeyframe);
        return keyframe;
    }

    Target CreateFlybyTarget(ProgramTarget corner, ProgramTarget next, ProgramTarget entry, ProgramTarget exit, double t)
    {
        return UseJointBlend(corner, next)
            ? CreateJointFlybyTarget(corner, entry, exit, t)
            : CreateCartesianFlybyTarget(corner, entry, exit, t);
    }

    JointTarget CreateJointFlybyTarget(ProgramTarget corner, ProgramTarget entry, ProgramTarget exit, double t)
    {
        var joints = GeometryUtil.Quadratic(entry.Kinematics.Joints, corner.Kinematics.Joints, exit.Kinematics.Joints, t);
        int robotJointCount = _robotSystem.GetRobotJointCount(corner.Group);
        var external = joints[robotJointCount..];
        return new JointTarget(joints[..robotJointCount], corner.Target, external);
    }

    CartesianTarget CreateCartesianFlybyTarget(ProgramTarget corner, ProgramTarget entry, ProgramTarget exit, double t)
    {
        Plane entryPlane = entry.WorldPlane;
        Plane cornerPlane = corner.WorldPlane;
        Plane exitPlane = exit.WorldPlane;

        Plane plane = _robotSystem.CartesianLerp(entryPlane, exitPlane, t, 0.0, 1.0)
            .WithOrigin(GeometryUtil.Quadratic(entryPlane.Origin, cornerPlane.Origin, exitPlane.Origin, t));
        plane = corner.ToTargetPlane(plane);

        var joints = GeometryUtil.Quadratic(entry.Kinematics.Joints, corner.Kinematics.Joints, exit.Kinematics.Joints, t);
        int robotJointCount = _robotSystem.GetRobotJointCount(corner.Group);
        var external = joints[robotJointCount..];
        return new CartesianTarget(plane, corner.Target, corner.Kinematics.Configuration, Motions.Linear, external);
    }

    static bool UseJointBlend(ProgramTarget corner, ProgramTarget next) =>
        corner.IsJointMotion || next.IsJointMotion;

    static int GetFlybyDivisions(SystemTarget corner, SystemTarget entry, SystemTarget exit, double stepSize)
    {
        double length = 0;

        for (int group = 0; group < corner.ProgramTargets.Count; group++)
        {
            if (!IsFlyby(corner.ProgramTargets[group]))
                continue;

            var entryPlane = entry.ProgramTargets[group].WorldPlane;
            var cornerPlane = corner.ProgramTargets[group].WorldPlane;
            var exitPlane = exit.ProgramTargets[group].WorldPlane;
            double currentLength = entryPlane.Origin.DistanceTo(cornerPlane.Origin) + cornerPlane.Origin.DistanceTo(exitPlane.Origin);
            length = Max(length, currentLength);
        }

        return Max(2, (int)Ceiling(length / stepSize));
    }

    static void SetKeyframeTiming(SystemTarget keyframe, double time, SystemTarget previous)
    {
        time = Max(time, previous.TotalTime);
        keyframe.TotalTime = time;
        keyframe.DeltaTime = time - previous.TotalTime;
        keyframe.MinTime = keyframe.DeltaTime;
    }

    static void AddKeyframe(List<SystemTarget> keyframes, SystemTarget keyframe) =>
        keyframes.Add(keyframe.ShallowClone());

    static SystemTarget ResetTiming(SystemTarget systemTarget)
    {
        systemTarget.DeltaTime = 0;
        systemTarget.TotalTime = 0;
        systemTarget.MinTime = 0;
        return systemTarget;
    }

    static bool ShouldStartNewKeyframe(IReadOnlyList<EndpointMode> modes, double deltaTime, double lastDeltaTime) =>
        modes.Contains(EndpointMode.Interpolated) || Abs(deltaTime - lastDeltaTime) > 1e-09;

    void AddWaitTime(SystemTarget systemTarget, SystemTarget prevStep, ref double time)
    {
        double wait = LongestWaitTime(systemTarget);

        if (wait <= TimeTol)
            return;

        time += wait;
        prevStep.TotalTime = time;
        prevStep.DeltaTime += wait;
        Keyframes.Add(prevStep.ShallowClone());
    }

    static double LongestWaitTime(SystemTarget systemTarget)
    {
        double longest = 0;

        foreach (var target in systemTarget.ProgramTargets)
        {
            double wait = target.Commands.OfType<Commands.Wait>().Sum(x => x.Seconds);
            longest = Max(longest, wait);
        }

        return longest;
    }

    static void FinalizeTarget(SystemTarget systemTarget, SystemTarget prevStep, double time, SegmentSpeed segmentTime)
    {
        systemTarget.TotalTime = time;
        systemTarget.DeltaTime = segmentTime.DeltaTime;
        systemTarget.MinTime = segmentTime.MinTime;

        foreach (var programTarget in systemTarget.ProgramTargets)
            CopyEndpoint(programTarget, prevStep.ProgramTargets[programTarget.Group]);
    }

    void TrackZoneCap(ProgramTarget target, string side, double requested, double used)
    {
        ZoneCap cap = new(target.Index, target.Group, side, requested, used);

        if (_zoneCapCount == 0)
        {
            _firstZoneCap = cap;
            _worstZoneCap = cap;
        }
        else if (CapDelta(cap) > CapDelta(_worstZoneCap))
        {
            _worstZoneCap = cap;
        }

        _zoneCapCount++;
    }

    void AddZoneCapWarnings()
    {
        if (_zoneCapCount == 0)
            return;

        var first = _firstZoneCap;
        var worst = _worstZoneCap;

        _program.AddWarning(
            IssueKind.MotionWarning,
            _zoneCapCount,
            first.Index,
            first.Group,
            nameof(ProgramMotionPlanner),
            () => $"Fly-by zone {first.Requested:0.###} mm at target {first.Index} of robot {first.Group} exceeds half of the {first.Side} segment; simulation uses {first.Used:0.###} mm on that side.",
            count => $"{count} fly-by zone segment(s) exceed half of an adjacent segment; simulation caps their effective blend distance. First affected target is {first.Index} of robot {first.Group}; largest cap uses {worst.Used:0.###} mm instead of {worst.Requested:0.###} mm at target {worst.Index} of robot {worst.Group}.");
    }

    static double CapDelta(ZoneCap cap) =>
        cap.Requested - cap.Used;

    void AddMotionWarnings()
    {
        AddMotionWarning(
            _stationaryCount,
            _firstStationary,
            first => $"Position and orientation do not change for target {first.Index}.",
            (count, first) => $"Position and orientation do not change for {count} target(s); first is target {first.Index} in robot {first.Group}.");

        AddMotionWarning(
            _rotationCount,
            _firstRotation,
            first => $"Rotation speed limit reached in target {first.Index}.",
            (count, first) => $"Rotation speed limit reached in {count} target(s); first is target {first.Index} in robot {first.Group}.");

        AddMotionWarning(
            _axisCount,
            _firstAxis,
            first => $"Axis {first.Joint + 1} speed limit reached in target {first.Index}.",
            (count, first) => $"Axis speed limit reached in {count} target(s); first is axis {first.Joint + 1} in target {first.Index} of robot {first.Group}.");

        AddMotionWarning(
            _externalCount,
            _firstExternal,
            first => $"External axis {first.Joint + 1} speed limit reached in target {first.Index}.",
            (count, first) => $"External axis speed limit reached in {count} target(s); first is axis {first.Joint + 1} in target {first.Index} of robot {first.Group}.");
    }

    void AddMotionWarning(int count, MotionWarn first, Func<MotionWarn, string> singular, Func<int, MotionWarn, string> plural)
        => _program.AddWarning(
            IssueKind.MotionWarning,
            count,
            first.Index,
            first.Group,
            nameof(ProgramMotionPlanner),
            () => singular(first),
            total => plural(total, first));

    static void CopyEndpoint(ProgramTarget target, ProgramTarget source)
    {
        target.Kinematics = source.Kinematics;
        target.ChangesConfiguration = source.ChangesConfiguration;
        target.LeadingJoint = source.LeadingJoint;
        target.SpeedType = source.SpeedType;
    }

    EndpointMode GetEndpointMode(ProgramTarget target)
    {
        if (!_robotSystem.RequiresContinuation(target.Group))
            return EndpointMode.Direct;

        return target.Target switch
        {
            CartesianTarget { Motion: Motions.Joint } => EndpointMode.Continuation,
            _ when target.IsJointMotion => EndpointMode.Direct,
            _ => EndpointMode.Interpolated
        };
    }

    double[][] SolveEndpoints(SystemTarget systemTarget, SystemTarget previous, double[][]? prevJoints, IReadOnlyList<EndpointMode> modes)
    {
        var targets = systemTarget.ProgramTargets.Map(programTarget =>
        {
            var prevTarget = previous.ProgramTargets[programTarget.Group];

            if (modes[programTarget.Group] != EndpointMode.Direct)
                return prevTarget.Target;

            if (programTarget.Target is CartesianTarget { Motion: Motions.Linear } cartesian)
                return new CartesianTarget(cartesian.Plane, cartesian, prevTarget.Kinematics.Configuration, Motions.Linear, cartesian.External);

            return programTarget.Target;
        });

        var kinematics = _robotSystem.Kinematics(targets, prevJoints);
        SetEndpoints(systemTarget, previous, kinematics, modes, EndpointMode.Direct);
        var mergedJoints = MergePreviousJoints(prevJoints, kinematics, modes, EndpointMode.Direct);

        if (!modes.Contains(EndpointMode.Continuation))
            return mergedJoints;

        var continuation = SolveContinuationEndpoint(systemTarget, previous, targets, mergedJoints, modes);
        SetEndpoints(systemTarget, previous, continuation, modes, EndpointMode.Continuation);
        return MergePreviousJoints(mergedJoints, continuation, modes, EndpointMode.Continuation);
    }

    void SetEndpoints(SystemTarget systemTarget, SystemTarget previous, IReadOnlyList<KinematicSolution> kinematics, IReadOnlyList<EndpointMode> modes, EndpointMode mode)
    {
        foreach (var target in systemTarget.ProgramTargets)
        {
            if (modes[target.Group] == mode)
                target.SetTargetKinematics(kinematics[target.Group], _program, previous.ProgramTargets[target.Group]);
        }
    }

    IReadOnlyList<KinematicSolution> SolveContinuationEndpoint(SystemTarget systemTarget, SystemTarget previous, IReadOnlyList<Target> baseTargets, double[][] prevJoints, IReadOnlyList<EndpointMode> modes)
    {
        var continuations = systemTarget.ProgramTargets
            .Where(target => modes[target.Group] == EndpointMode.Continuation)
            .ToArray();

        var divisions = continuations.Map(target =>
        {
            var prevTarget = previous.ProgramTargets[target.Group];
            double distance = target.GetPrevPlane(prevTarget).Origin.DistanceTo(target.Plane.Origin);
            return Max(1, (int)Ceiling(distance / ContinuationStep));
        });

        int maxSteps = divisions.Max();
        IReadOnlyList<KinematicSolution>? kinematics = null;

        for (int step = 1; step <= maxSteps; step++)
        {
            var targets = baseTargets.Map(target => target);

            for (int i = 0; i < continuations.Length; i++)
            {
                var target = continuations[i];
                var prevTarget = previous.ProgramTargets[target.Group];
                double t = Min(1.0, step / (double)divisions[i]);

                var prevPlane = target.GetPrevPlane(prevTarget);
                var plane = _robotSystem.CartesianLerp(prevPlane, target.Plane, t, 0.0, 1.0);
                var external = LerpExternal(prevTarget, target, t);
                targets[target.Group] = new CartesianTarget(plane, target.Target, configuration: null, Motions.Joint, external);
            }

            kinematics = _robotSystem.Kinematics(targets, prevJoints);
            prevJoints = kinematics.Map(k => k.Joints);

            if (kinematics.Any(k => k.Errors.Count > 0))
                break;
        }

        return kinematics.NotNull();
    }

    double[] LerpExternal(ProgramTarget prevTarget, ProgramTarget target, double t)
    {
        var end = target.Target.External;

        if (end.Length == 0)
            return [];

        var start = new double[end.Length];

        if (prevTarget.Target.External.Length == end.Length)
        {
            start = prevTarget.Target.External;
        }
        else if (end.Length == 1 && _robotSystem.RedundantJointIndex(target.Group) is int index)
        {
            start[0] = prevTarget.Kinematics.Joints[index];
        }

        return JointTarget.Lerp(start, end, t, 0.0, 1.0);
    }

    static double[][] MergePreviousJoints(double[][]? previous, IReadOnlyList<KinematicSolution> kinematics, IReadOnlyList<EndpointMode> modes, EndpointMode mode)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(modes.Count, kinematics.Count, nameof(modes));

        var result = new double[kinematics.Count][];

        if (previous is not null)
        {
            ArgumentOutOfRangeException.ThrowIfNotEqual(previous.Length, result.Length, nameof(previous));

            for (int i = 0; i < previous.Length; i++)
                result[i] = previous[i] ?? throw new ArgumentException($"Previous joints for group {i} are missing.");
        }

        for (int group = 0; group < result.Length; group++)
        {
            if (modes[group] == mode || result[group] is null)
                result[group] = kinematics[group].Joints;
        }

        return result;
    }

    void SetInterpolatedEndpoint(SystemTarget systemTarget, SystemTarget prevStep, IReadOnlyList<EndpointMode> modes)
    {
        foreach (var programTarget in systemTarget.ProgramTargets)
        {
            if (modes[programTarget.Group] != EndpointMode.Interpolated)
                continue;

            programTarget.SetTargetKinematics(prevStep.ProgramTargets[programTarget.Group].Kinematics, _program, prevStep.ProgramTargets[programTarget.Group]);
        }
    }

    void CheckUndefined(SystemTarget systemTarget, List<SystemTarget> systemTargets)
    {
        if (systemTarget.Index < systemTargets.Count - 1)
        {
            int i = systemTarget.Index;
            foreach (var target in systemTarget.ProgramTargets.Where(x => x.Kinematics.Configuration == RobotConfigurations.Undefined))
            {
                if (!systemTargets[i + 1].ProgramTargets[target.Group].IsJointMotion)
                {
                    _program.AddError(IssueKind.KinematicError, $"Undefined configuration (probably due to a singularity) in target {target.Index} of robot {target.Group} before a linear motion.", target.Index, target.Group, nameof(ProgramMotionPlanner));
                }
            }
        }
    }

    SegmentSpeed GetSegmentSpeed(SystemTarget systemTarget, SystemTarget previous, double timeScale)
    {
        double slowestDelta = 0;
        double slowestMinTime = 0;

        foreach (var target in systemTarget.ProgramTargets)
        {
            var speed = GetTargetSpeed(target, previous.ProgramTargets[target.Group], timeScale);
            slowestDelta = Max(slowestDelta, speed.DeltaTime);
            slowestMinTime = Max(slowestMinTime, speed.MinTime);
            target.LeadingJoint = speed.LeadingJoint;
            target.SpeedType = speed.Type;
        }

        return new(slowestDelta, slowestMinTime);
    }

    TargetSpeed GetTargetSpeed(ProgramTarget target, ProgramTarget prevTarget, double timeScale)
    {
        Plane prevPlane = target.GetPrevPlane(prevTarget);
        var joints = _robotSystem.GetJoints(target.Group);
        var axis = GetAxisSpeed(target, prevTarget, joints);
        var external = GetExternalSpeed(target, prevTarget, joints);
        var deltaTimes = GetDeltaTimes(target, prevPlane, axis.Time, external.Time, timeScale);
        var delta = MaxDelta(deltaTimes);
        int leadingJoint = axis.LeadingJoint;
        var speedType = ClassifySpeed(target, delta.Time, delta.Index, external.LeadingJoint, ref leadingJoint);

        return new(delta.Time, axis.Time, leadingJoint, speedType);
    }

    static JointSpeed GetAxisSpeed(ProgramTarget target, ProgramTarget prevTarget, IReadOnlyList<Joint> joints)
    {
        double deltaTime = 0;
        int leadingJoint = -1;

        for (int i = 0; i < target.Kinematics.Joints.Length; i++)
        {
            double currentTime = Abs(target.Kinematics.Joints[i] - prevTarget.Kinematics.Joints[i]) / joints[i].MaxSpeed;

            if (currentTime > deltaTime)
            {
                deltaTime = currentTime;
                leadingJoint = i;
            }
        }

        return new(deltaTime, leadingJoint);
    }

    JointSpeed GetExternalSpeed(ProgramTarget target, ProgramTarget prevTarget, IReadOnlyList<Joint> joints)
    {
        double deltaTime = 0;
        int leadingJoint = -1;
        int jointCount = _robotSystem.GetRobotJointCount(target.Group);
        int externalCount = _robotSystem.GetExternalJointCount(target.Group);

        for (int i = 0; i < externalCount; i++)
        {
            int jointIndex = i + jointCount;
            var joint = joints[jointIndex];
            double jointSpeed = joint.MaxSpeed;

            if (joint is PrismaticJoint)
            {
                jointSpeed = Min(jointSpeed, target.Target.Speed.TranslationExternal);
            }
            else if (joint is RevoluteJoint)
            {
                jointSpeed = Min(jointSpeed, target.Target.Speed.RotationExternal);
            }

            double currentTime = Abs(target.Kinematics.Joints[jointIndex] - prevTarget.Kinematics.Joints[jointIndex]) / jointSpeed;

            if (currentTime > deltaTime)
            {
                deltaTime = currentTime;
                leadingJoint = jointIndex;
            }
        }

        return new(deltaTime, leadingJoint);
    }

    static Vector6d GetDeltaTimes(ProgramTarget target, Plane prevPlane, double axisTime, double externalTime, double timeScale)
    {
        var speed = target.Target.Speed;

        if (speed.Time > 0)
            return new(speed.Time * timeScale, 0, 0, 0, 0, 0);

        double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
        double linearTime = distance / speed.TranslationSpeed;
        double angleSwivel = Vector3d.VectorAngle(prevPlane.Normal, target.Plane.Normal);
        double angleRotation = Vector3d.VectorAngle(prevPlane.XAxis, target.Plane.XAxis);
        double rotationTime = Max(angleSwivel, angleRotation) / speed.RotationSpeed;

        return new(linearTime, rotationTime, axisTime, externalTime, 0, 0);
    }

    static (int Index, double Time) MaxDelta(Vector6d values)
    {
        double max = 0;
        int index = -1;

        for (int i = 0; i < 4; i++)
        {
            if (values[i] > max)
            {
                max = values[i];
                index = i;
            }
        }

        return (index, max);
    }

    SpeedType ClassifySpeed(ProgramTarget target, double deltaTime, int deltaIndex, int externalLeadingJoint, ref int leadingJoint)
    {
        if (deltaTime < TimeTol)
        {
            TrackMotion(ref _stationaryCount, ref _lastStationary, ref _firstStationary, target);
            return SpeedType.Tcp;
        }

        if (deltaIndex == 1)
        {
            TrackMotion(ref _rotationCount, ref _lastRotation, ref _firstRotation, target);
            return SpeedType.Rotation;
        }

        if (deltaIndex == 2)
        {
            if (leadingJoint < 0)
                throw new InvalidOperationException("Axis speed limit was reached, but no leading joint was found.");

            TrackMotion(ref _axisCount, ref _lastAxis, ref _firstAxis, target, leadingJoint);
            return SpeedType.Axis;
        }

        if (deltaIndex == 3)
        {
            if (externalLeadingJoint < 0)
                throw new InvalidOperationException("External axis speed limit was reached, but no leading external joint was found.");

            leadingJoint = externalLeadingJoint;
            TrackMotion(ref _externalCount, ref _lastExternal, ref _firstExternal, target, externalLeadingJoint);
            return SpeedType.External;
        }

        return SpeedType.Tcp;
    }

    static void TrackMotion(ref int count, ref int last, ref MotionWarn first, ProgramTarget target, int joint = -1)
    {
        if (target.Index == last)
            return;

        last = target.Index;

        if (count == 0)
            first = new(target.Index, target.Group, joint);

        count++;
    }
}
