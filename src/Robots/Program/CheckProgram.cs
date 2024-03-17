using Rhino.Geometry;
using static Robots.Util;
using static System.Math;

namespace Robots;

class CheckProgram
{
    readonly Program _program;
    readonly RobotSystem _robotSystem;
    int _lastIndex;

    internal List<SystemTarget> Keyframes { get; } = [];
    internal List<SystemTarget> FixedTargets { get; }

    internal CheckProgram(Program program, List<SystemTarget> systemTarget, double stepSize)
    {
        _robotSystem = program.RobotSystem;
        _program = program;

        FixFirstTarget(systemTarget[0]);
        FixTargetAttributes(systemTarget);
        var indexError = FixTargetMotions(systemTarget, stepSize);

        FixedTargets = (indexError != -1)
             ? systemTarget.GetRange(0, indexError + 1)
             : systemTarget;
    }

    void FixFirstTarget(SystemTarget firstTarget)
    {
        var fix = firstTarget.ProgramTargets.Where(x => !x.IsJointTarget);

        if (fix.Any())
        {
            var kinematics = _robotSystem.Kinematics(firstTarget.ProgramTargets.Select(x => x.Target));

            foreach (var programTarget in fix)
            {
                var kinematic = kinematics[programTarget.Group];
                if (kinematic.Errors.Count > 0)
                {
                    _program.Errors.Add($"Errors in target {programTarget.Index} of robot {programTarget.Group}:");
                    _program.Errors.AddRange(kinematic.Errors);
                }

                programTarget.Target = new JointTarget(kinematic.Joints.RangeSubset(0, _robotSystem.RobotJointCount), programTarget.Target);
                _program.Warnings.Add($"First target in robot {programTarget.Group} changed to a joint motion using axis rotations");
            }
        }
    }

    void FixTargetAttributes(List<SystemTarget> systemTargets)
    {
        var allProgramTargets = systemTargets.SelectMany(x => x.ProgramTargets);
        var allTargets = allProgramTargets.Select(t => t.Target);

        // Fix externals
        {
            if (_robotSystem is IndustrialSystem industrialSystem)
            {
                int resizeCount = 0;
                ProgramTarget? resizeTarget = null;

                foreach (var systemTarget in systemTargets)
                {
                    foreach (var programTarget in systemTarget.ProgramTargets)
                    {
                        var mechGroup = industrialSystem.MechanicalGroups[programTarget.Group];
                        int externalCount = mechGroup.Externals.Sum(e => e.Joints.Length);

                        if (programTarget.Target.External.Length != externalCount)
                        {
                            //double[] external = programTarget.Target.External;
                            //Array.Resize(ref external, externalCount);
                            //programTarget.Target.External = external;
                            resizeCount++;
                            resizeTarget ??= programTarget;
                        }
                    }
                }

                if (resizeTarget is not null)
                {
                    _program.Warnings.Add($"{resizeCount} targets have wrong number of external axes configured, the first one being target {resizeTarget.Index} of robot {resizeTarget.Group}.");
                }
            }
        }

        // Warn about defaults

        void CheckTargets(Func<Target, bool> condition, string message, Action<ProgramTarget>? action = null)
        {
            var results = allProgramTargets.Where(x => condition(x.Target)).ToList();

            if (results.Count > 0)
            {
                var first = results[0];
                _program.Warnings.Add($"{results.Count} targets {message}, the first one being target {first.Index} in robot {first.Group}");

                if (action is not null)
                {
                    foreach (var result in results)
                        action(result);
                }
            }
        }

        CheckTargets(t => t.Tool.Equals(Tool.Default), "have their tool set to default");
        CheckTargets(t => t.Speed.Equals(Speed.Default), "have their speed set to default");

        // Fix linear with forced config

        CheckTargets(
            t => t is CartesianTarget cartesian && cartesian.Motion == Motions.Linear && cartesian.Configuration is not null,
            "are set to linear with a forced configuration (configuration is ignored for linear motions)",
            t => ((CartesianTarget)t.Target).Configuration = null
            );

        // Warn max payload
        var tools = allProgramTargets.Select(y => (y.Target.Tool, y.Group)).Distinct();

        foreach (var (tool, group) in tools)
        {
            double payload = _robotSystem.Payload(group);
            if (tool.Weight > payload) _program.Warnings.Add($"Weight of tool {tool.Name} exceeds the robot {group} rated payload of {payload} kg");
        }

        // Get unique attributes
        HashSet<TargetAttribute> attributes = [];

        foreach (var target in allTargets)
        {
            attributes.Add(target.Tool);
            attributes.Add(target.Frame);
            attributes.Add(target.Speed);
            attributes.Add(target.Zone);
        }

        List<Command> commands = [.. _program.InitCommands, .. allProgramTargets.SelectMany(y => y.Commands)];

        foreach (var command in commands)
            attributes.Add(command);

        // Name attributes with no name
        {
            Dictionary<Type, int> types = [];

            foreach (var attribute in attributes.ToList())
            {
                if (!attribute.HasName)
                {
                    var type = attribute.GetType();

                    if (!types.TryGetValue(type, out int i))
                        i = 0;

                    types[type] = ++i;

                    string name = $"{type.Name}{i - 1:000}";
                    SetAttributeName(attribute, attributes, allProgramTargets, name);
                }
            }
        }

        // Rename attributes with duplicate names
        {
            var duplicates = attributes.GroupBy(a => a.Name);

            foreach (var group in duplicates)
            {
                if (!group.Skip(1).Any())
                    continue;

                _program.Warnings.Add($"Multiple target attributes named \"{group.Key}\" found");
                int i = 0;

                foreach (var attribute in group)
                {
                    string name = $"{attribute.Name}{i++:000}";
                    SetAttributeName(attribute, attributes, allProgramTargets, name);
                }
            }
        }

        // Fix frames
        {
            foreach (var frame in attributes.OfType<Frame>())
            {
                if (frame.CoupledMechanicalGroup == -1 && frame.CoupledMechanism != -1)
                {
                    throw new ArgumentException($" Frame {frame.Name} has a coupled mechanism set but no mechanical group.");
                }

                if (frame.CoupledMechanicalGroup == 0 && frame.CoupledMechanism == -1)
                {
                    throw new ArgumentException($" Frame {frame.Name} is set to couple the robot rather than a mechanism.");
                }

                if (frame.IsCoupled && _robotSystem is IndustrialSystem industrialSystem)
                {
                    if (frame.CoupledMechanicalGroup > industrialSystem.MechanicalGroups.Count - 1)
                    {
                        throw new ArgumentException($" Frame {frame.Name} is set to couple an inexistent mechanical group.");
                    }

                    if (frame.CoupledMechanism > industrialSystem.MechanicalGroups[frame.CoupledMechanicalGroup].Externals.Count - 1)
                    {
                        throw new ArgumentException($" Frame {frame.Name} is set to couple an inexistent mechanism.");
                    }

                    frame.CoupledPlaneIndex = ((IndustrialSystem)_robotSystem).GetPlaneIndex(frame);
                }
            }
        }

        _program.Attributes.AddRange(attributes);
    }

    int FixTargetMotions(List<SystemTarget> systemTargets, double stepSize)
    {
        double time = 0;
        //int groups = systemTargets[0].ProgramTargets.Count;
        double[][]? prevJoints = null;
        bool is7dof = _robotSystem.RobotJointCount == 7;

        for (int i = 0; i < systemTargets.Count; i++)
        {
            var systemTarget = systemTargets[i];
            //int errorIndex = -1;

            // first target
            if (i == 0)
            {
                var firstKinematics = _robotSystem.Kinematics(systemTarget.ProgramTargets.Select(x => x.Target));
                prevJoints = firstKinematics.Map(k => k.Joints);
                systemTarget.SetTargetKinematics(firstKinematics, _program.Errors, _program.Warnings);
                CheckUndefined(systemTarget, systemTargets);
                Keyframes.Add(systemTarget.ShallowClone());
            }
            else
            {
                var prevSystemTarget = systemTargets[i - 1];

                // no interpolation
                {
                    var kineTargets = systemTarget.ProgramTargets.MapToList(x => x.Target.ShallowClone());

                    for (int j = 0; j < kineTargets.Count; j++)
                    {
                        if (kineTargets[j] is CartesianTarget target && target.Motion == Motions.Linear)
                        {
                            target.Configuration = prevSystemTarget.ProgramTargets[j].Kinematics.Configuration;
                        }
                    }

                    if (!is7dof)
                    {
                        var kinematics = _robotSystem.Kinematics(kineTargets, prevJoints);
                        prevJoints = kinematics.Map(k => k.Joints);
                        systemTarget.SetTargetKinematics(kinematics, _program.Errors, _program.Warnings, prevSystemTarget);
                    }
                }

                double divisions = 1;
                var linearTargets = systemTarget.ProgramTargets.Where(x => !x.IsJointMotion).ToList();
                //   if (linearTargets.Count() > 0) program.Errors.Clear();

                foreach (var target in linearTargets)
                {
                    var prevProgramTarget = prevSystemTarget.ProgramTargets[target.Group];
                    var prevPlane = target.GetPrevPlane(prevProgramTarget);
                    double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
                    double divisionsTemp = Ceiling(distance / stepSize);
                    if (divisionsTemp > divisions) divisions = divisionsTemp;
                }

                var prevInterTarget = prevSystemTarget.ShallowClone();
                prevInterTarget.DeltaTime = 0;
                prevInterTarget.TotalTime = 0;
                prevInterTarget.MinTime = 0;

                double totalDeltaTime = 0;
                double lastDeltaTime = 0;
                double deltaTimeSinceLast = 0;
                double totalMinTime = 0;
                double minTimeSinceLast = 0;

                for (double j = 1; j <= divisions; j++)
                {
                    double t = j / divisions;
                    var interTarget = systemTarget.ShallowClone();
                    var kineTargets = systemTarget.Lerp(prevSystemTarget, _robotSystem, t, 0.0, 1.0);
                    var kinematics = _program.RobotSystem.Kinematics(kineTargets, prevJoints);
                    prevJoints = kinematics.Map(k => k.Joints);
                    interTarget.SetTargetKinematics(kinematics, _program.Errors, _program.Warnings, prevInterTarget);

                    // Set speed

                    double slowestDelta = 0;
                    double slowestMinTime = 0;
                    foreach (var target in interTarget.ProgramTargets)
                    {
                        var speeds = GetSpeeds(target, prevInterTarget.ProgramTargets[target.Group]);
                        slowestDelta = Max(slowestDelta, speeds.Item1);
                        slowestMinTime = Max(slowestMinTime, speeds.Item2);
                        target.LeadingJoint = speeds.Item3;
                        target.SpeedType = speeds.Item4;
                    }

                    if ((j > 1) && (is7dof || (Abs(slowestDelta - lastDeltaTime) > 1e-09)))
                    {
                        Keyframes.Add(prevInterTarget.ShallowClone());
                        deltaTimeSinceLast = 0;
                        minTimeSinceLast = 0;
                    }

                    lastDeltaTime = slowestDelta;
                    time += slowestDelta;
                    totalDeltaTime += slowestDelta;
                    deltaTimeSinceLast += slowestDelta;
                    totalMinTime += slowestMinTime;
                    minTimeSinceLast += slowestMinTime;

                    interTarget.DeltaTime = deltaTimeSinceLast;
                    interTarget.MinTime = minTimeSinceLast;
                    interTarget.TotalTime = time;

                    prevInterTarget = interTarget;
                }

                Keyframes.Add(prevInterTarget.ShallowClone());

                if (_program.Errors.Count == 0)
                {
                    double longestWaitTime = 0;
                    foreach (var target in systemTarget.ProgramTargets)
                    {
                        double waitTime = target.Commands.OfType<Commands.Wait>().Select(x => x.Seconds).Sum();
                        if (waitTime > longestWaitTime) longestWaitTime = waitTime;
                    }

                    if (longestWaitTime > TimeTol)
                    {
                        time += longestWaitTime;
                        totalDeltaTime += longestWaitTime;

                        prevInterTarget.TotalTime = time;
                        prevInterTarget.DeltaTime += longestWaitTime;
                        Keyframes.Add(prevInterTarget.ShallowClone());
                    }
                }

                // set target kinematics
                if (is7dof)
                {
                    var lastKinematics = prevInterTarget.ProgramTargets.MapToList(p => p.Kinematics);
                    systemTarget.SetTargetKinematics(lastKinematics, _program.Errors, _program.Warnings, prevInterTarget);
                }

                CheckUndefined(systemTarget, systemTargets);

                systemTarget.TotalTime = time;
                systemTarget.DeltaTime = totalDeltaTime;
                systemTarget.MinTime = totalMinTime;

                foreach (var programTarget in systemTarget.ProgramTargets)
                {
                    int group = programTarget.Group;
                    programTarget.Kinematics = prevInterTarget.ProgramTargets[group].Kinematics;
                    programTarget.ChangesConfiguration = prevInterTarget.ProgramTargets[group].ChangesConfiguration;
                    programTarget.LeadingJoint = prevInterTarget.ProgramTargets[group].LeadingJoint;
                    programTarget.SpeedType = prevInterTarget.ProgramTargets[group].SpeedType;
                }
            }

            if (_program.Errors.Count > 0)
            {
                _program.Duration = time;
                return i;
            }

            //   if (errorIndex != -1) return errorIndex;
        }

        _program.Duration = time;
        return -1;
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
                    _program.Errors.Add($"Undefined configuration (probably due to a singularity) in target {target.Index} of robot {target.Group} before a linear motion");
                    //IndexError = i;
                }
            }
        }
    }

    void SetAttributeName(TargetAttribute attribute, HashSet<TargetAttribute> attributes, IEnumerable<ProgramTarget> targets, string name)
    {
        var namedAttribute = attribute.CloneWithName<TargetAttribute>(name);

        attributes.Remove(attribute);
        attributes.Add(namedAttribute);

        //int index = _program.Attributes.FindIndex(x => x == attribute);
        //_program.Attributes[index] = namedAttribute;

        if (namedAttribute is Tool tool)
        {
            foreach (var target in targets) if (target.Target.Tool.Equals(attribute)) { target.Target = target.Target.ShallowClone(); target.Target.Tool = tool; }
        }
        else if (namedAttribute is Frame frame)
        {
            foreach (var target in targets) if (target.Target.Frame.Equals(attribute)) { target.Target = target.Target.ShallowClone(); target.Target.Frame = frame; }
        }
        else if (namedAttribute is Speed speed)
        {
            foreach (var target in targets) if (target.Target.Speed.Equals(attribute)) { target.Target = target.Target.ShallowClone(); target.Target.Speed = speed; }
        }
        else if (namedAttribute is Zone zone)
        {
            foreach (var target in targets) if (target.Target.Zone.Equals(attribute)) { target.Target = target.Target.ShallowClone(); target.Target.Zone = zone; }
        }
        else if (namedAttribute is Command command)
        {
            for (int i = 0; i < _program.InitCommands.Count; i++)
                if (_program.InitCommands[i].Equals(attribute)) _program.InitCommands[i] = command;

            foreach (var target in targets)
            {
                var group = target.Commands.NotNull();

                for (int i = 0; i < group.Count; i++)
                {
                    if (group[i].Equals(attribute))
                        group[i] = command;
                }
            }
        }
    }

    (double, double, int, SpeedType) GetSpeeds(ProgramTarget target, ProgramTarget prevTarget)
    {
        Plane prevPlane = target.GetPrevPlane(prevTarget);
        var joints = _robotSystem.GetJoints(target.Group);
        double deltaTime = 0;

        // Axis
        double deltaAxisTime = 0;
        int leadingJoint = -1;

        for (int i = 0; i < target.Kinematics.Joints.Length; i++)
        {
            double deltaCurrentAxisTime = Abs(target.Kinematics.Joints[i] - prevTarget.Kinematics.Joints[i]) / joints[i].MaxSpeed;

            if (deltaCurrentAxisTime > deltaAxisTime)
            {
                deltaAxisTime = deltaCurrentAxisTime;
                leadingJoint = i;
            }
        }

        // External
        double deltaExternalTime = 0;
        int externalLeadingJoint = -1;
        int deltaIndex = -1;
        int jointCount = _robotSystem.RobotJointCount;

        double externalCount = 0;

        if (_robotSystem is IndustrialSystem industrialSystem)
            externalCount = industrialSystem.MechanicalGroups[target.Group].Externals.Sum(e => e.Joints.Length);

        for (int i = 0; i < externalCount; i++)
        {
            var joint = joints[i + jointCount];
            double jointSpeed = joint.MaxSpeed;
            if (joint is PrismaticJoint) jointSpeed = Min(jointSpeed, target.Target.Speed.TranslationExternal);
            else if (joint is RevoluteJoint) jointSpeed = Min(jointSpeed, target.Target.Speed.RotationExternal);

            double deltaCurrentExternalTime = Abs(target.Kinematics.Joints[i + jointCount] - prevTarget.Kinematics.Joints[i + jointCount]) / jointSpeed;

            if (deltaCurrentExternalTime > deltaExternalTime)
            {
                deltaExternalTime = deltaCurrentExternalTime;
                externalLeadingJoint = i + jointCount;
            }
        }

        Vector6d deltaTimes;

        if (target.Target.Speed.Time > 0)
        {
            // Get slowest by time
            double deltaTimeTime = target.Target.Speed.Time;
            deltaTimes = new Vector6d(deltaTimeTime, double.MaxValue, deltaAxisTime, deltaExternalTime, 0, 0);
        }
        else
        {
            // Translation
            double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
            double deltaLinearTime = distance / target.Target.Speed.TranslationSpeed;

            // Rotation
            double angleSwivel = Vector3d.VectorAngle(prevPlane.Normal, target.Plane.Normal);
            double angleRotation = Vector3d.VectorAngle(prevPlane.XAxis, target.Plane.XAxis);
            double angle = Max(angleSwivel, angleRotation);
            double deltaRotationTime = angle / target.Target.Speed.RotationSpeed;

            // Get slowest
            deltaTimes = new Vector6d(deltaLinearTime, deltaRotationTime, deltaAxisTime, deltaExternalTime, 0, 0);
        }

        for (int i = 0; i < 4; i++)
        {
            if (deltaTimes[i] > deltaTime)
            {
                deltaTime = deltaTimes[i];
                deltaIndex = i;
            }
        }

        var speedType = SpeedType.Tcp;

        if (deltaTime < TimeTol)
        {
            var text = $"Position and orientation do not change for target {target.Index}";
            if (!_program.Warnings.Contains(text))
                _program.Warnings.Add(text);
        }
        else if (deltaIndex == 1)
        {
            if (target.Index != _lastIndex) _program.Warnings.Add($"Rotation speed limit reached in target {target.Index}");
            _lastIndex = target.Index;
            speedType = SpeedType.Rotation;
        }
        else if (deltaIndex == 2)
        {
            if (target.Index != _lastIndex) _program.Warnings.Add($"Axis {leadingJoint + 1} speed limit reached in target {target.Index}");
            _lastIndex = target.Index;
            speedType = SpeedType.Axis;
        }
        else if (deltaIndex == 3)
        {
            if (target.Index != _lastIndex) _program.Warnings.Add($"External axis {externalLeadingJoint + 1} speed limit reached in target {target.Index}");
            leadingJoint = externalLeadingJoint;
            _lastIndex = target.Index;
            speedType = SpeedType.External;
        }

        return (deltaTime, deltaAxisTime, leadingJoint, speedType);
    }
}
