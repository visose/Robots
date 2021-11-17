using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using static Robots.Util;
using static System.Math;

namespace Robots
{
    class CheckProgram
    {
        readonly Program program;
        readonly RobotSystem robotSystem;
        internal List<CellTarget> keyframes = new List<CellTarget>();
        int lastIndex;
        internal int indexError;
        //readonly int groupCount;

        internal CheckProgram(Program program, List<CellTarget> cellTargets, double stepSize)
        {
            this.robotSystem = program.RobotSystem;
            this.program = program;
           // this.groupCount = cellTargets[0].ProgramTargets.Count;

            FixFirstTarget(cellTargets[0]);
            FixTargetAttributes(cellTargets);
            this.indexError = FixTargetMotions(cellTargets, stepSize);
        }

        void FixFirstTarget(CellTarget firstTarget)
        {
            var fix = firstTarget.ProgramTargets.Where(x => !x.IsJointTarget);

            if (fix.Any())
            {
                var kinematics = robotSystem.Kinematics(firstTarget.ProgramTargets.Select(x => x.Target));

                foreach (var programTarget in fix)
                {
                    var kinematic = kinematics[programTarget.Group];
                    if (kinematic.Errors.Count > 0)
                    {
                        program.Errors.Add($"Errors in target {programTarget.Index} of robot {programTarget.Group}:");
                        program.Errors.AddRange(kinematic.Errors);
                    }

                    programTarget.Target = new JointTarget(kinematic.Joints, programTarget.Target);
                    program.Warnings.Add($"First target in robot {programTarget.Group} changed to a joint motion using axis rotations");
                }
            }
        }

        void FixTargetAttributes(List<CellTarget> cellTargets)
        {
            // Fix externals

            int resizeCount = 0;
            ProgramTarget resizeTarget = null;

            foreach (var cellTarget in cellTargets)
            {
                foreach (var programTarget in cellTarget.ProgramTargets)
                {
                    int externalCount = 0;
                    if (robotSystem is RobotCell cell)
                        externalCount = cell.MechanicalGroups[programTarget.Group].Joints.Count - 6;

                    if (programTarget.Target.External.Length != externalCount)
                    {
                        double[] external = programTarget.Target.External;
                        Array.Resize<double>(ref external, externalCount);
                        programTarget.Target.External = external;
                        resizeCount++;
                        if (resizeTarget == null) resizeTarget = programTarget;
                    }
                }
            }

            if (resizeCount > 0)
            {
                program.Warnings.Add($"{resizeCount} targets had wrong number of external axes configured, the first one being target {resizeTarget.Index} of robot {resizeTarget.Group}.");
            }

            // Warn about defaults

            var defaultTools = cellTargets.SelectMany(x => x.ProgramTargets).Where(x => x.Target.Tool == Tool.Default).ToList();
            if (defaultTools.Count > 0)
            {
                var first = defaultTools[0];
                program.Warnings.Add($" {defaultTools.Count} targets have their tool set to default, the first one being target {first.Index} in robot {first.Group}");
            }

            var defaultSpeeds = cellTargets.SelectMany(x => x.ProgramTargets).Where(x => x.Target.Speed == Speed.Default).ToList();
            if (defaultSpeeds.Count > 0)
            {
                var first = defaultSpeeds[0];
                program.Warnings.Add($" {defaultSpeeds.Count} targets have their speed set to default, the first one being target {first.Index} in robot {first.Group}");
            }

            var linearForced = cellTargets.SelectMany(x => x.ProgramTargets).Where(x => x.Target is CartesianTarget cartesian && cartesian.Motion == Motions.Linear && cartesian.Configuration != null).ToList();
            if (linearForced.Count > 0)
            {
                var first = linearForced[0];
                program.Warnings.Add($" {linearForced.Count} targets are set to linear with a forced configuration, the first one being target {first.Index} in robot {first.Group}. Configuration setting is ignored for linear motions.");
            }

            foreach (var target in linearForced)
            {
                var cartesian = target.Target as CartesianTarget;
                cartesian.Configuration = null;
            }

            // Check max payload

            var tools = cellTargets.SelectMany(x => x.ProgramTargets.Select(y => new { y.Target.Tool, y.Group })).Distinct();
            foreach (var tool in tools)
            {
                double payload = robotSystem.Payload(tool.Group);
                if (tool.Tool.Weight > payload) program.Warnings.Add($"Weight of tool {tool.Tool.Name} exceeds the robot {tool.Group} rated payload of {payload} kg");
            }

            // Get unique attributes

            program.Attributes.AddRange(tools.Select(x => x.Tool).Distinct());
            program.Attributes.AddRange(cellTargets.SelectMany(x => x.ProgramTargets.Select(y => y.Target.Frame)).Distinct());
            program.Attributes.AddRange(cellTargets.SelectMany(x => x.ProgramTargets.Select(y => y.Target.Speed)).Distinct());
            program.Attributes.AddRange(cellTargets.SelectMany(x => x.ProgramTargets.Select(y => y.Target.Zone)).Distinct());

            var commands = new List<Command>();
            commands.AddRange(program.InitCommands);
            commands.AddRange(cellTargets.SelectMany(x => x.ProgramTargets.SelectMany(y => y.Commands)));
            program.Attributes.AddRange(commands.Distinct());

            // Name attributes with no name
            {
                var types = new List<Type>();
                foreach (var attribute in program.Attributes.ToList())
                {
                    if (attribute.Name == null)
                    {
                        var type = attribute.GetType();
                        types.Add(type);
                        int i = types.FindAll(x => x == type).Count;
                        string name = $"{type.Name}{i - 1:000}";
                        SetAttributeName(attribute, cellTargets.SelectMany(x => x.ProgramTargets), name);
                    }
                }
            }

            // Rename attributes with duplicate names
            {
                var duplicates = program.Attributes.GroupBy(x => x.Name).Where(x => x.Count() > 1);
                foreach (var group in duplicates)
                {
                    program.Warnings.Add($"Multiple target attributes named \"{group.Key}\" found");
                    int i = 0;
                    foreach (var attribute in group)
                    {
                        string name = $"{attribute.Name}{i++:000}";
                        SetAttributeName(attribute, cellTargets.SelectMany(x => x.ProgramTargets), name);
                    }
                }
            }

            // Fix frames
            {
                foreach (var frame in program.Attributes.OfType<Frame>())
                {
                    if (frame.CoupledMechanicalGroup == -1 && frame.CoupledMechanism != -1)
                    {
                        throw new Exception($" Frame {frame.Name} has a coupled mechanism set but no mechanical group.");
                    }

                    if (frame.CoupledMechanicalGroup == 0 && frame.CoupledMechanism == -1)
                    {
                        throw new Exception($" Frame {frame.Name} is set to couple the robot rather than a mechanism.");
                    }

                    if (frame.IsCoupled)
                    {
                        var cell = robotSystem as RobotCell;
                        if (frame.CoupledMechanicalGroup > cell.MechanicalGroups.Count - 1)
                        {
                            throw new Exception($" Frame {frame.Name} is set to couple an inexistant mechanical group.");
                        }

                        if (frame.CoupledMechanism > cell.MechanicalGroups[frame.CoupledMechanicalGroup].Externals.Count - 1)
                        {
                            throw new Exception($" Frame {frame.Name} is set to couple an inexistant mechanism.");
                        }

                        frame.CoupledPlaneIndex = ((RobotCell)robotSystem).GetPlaneIndex(frame);
                    }
                }
            }
        }

        int FixTargetMotions(List<CellTarget> cellTargets, double stepSize)
        {
            double time = 0;
            int groups = cellTargets[0].ProgramTargets.Count;

            for (int i = 0; i < cellTargets.Count; i++)
            {
                var cellTarget = cellTargets[i];
                int errorIndex = -1;

                // first target
                if (i == 0)
                {
                    var firstKinematics = robotSystem.Kinematics(cellTargets[0].ProgramTargets.Select(x => x.Target));
                    cellTargets[0].SetTargetKinematics(firstKinematics, program.Errors, program.Warnings);
                    CheckUndefined(cellTarget, cellTargets);
                    keyframes.Add(cellTargets[0].ShallowClone());
                    if (program.Errors.Count > 0) { errorIndex = i; }
                }
                else
                {
                    var prevTarget = cellTargets[i - 1];
                    var prevJoints = prevTarget.ProgramTargets.Select(x => x.Kinematics.Joints);

                    // no interpolation
                    {
                        var kineTargets = cellTarget.ProgramTargets.Select(x => x.Target.ShallowClone()).ToList();

                        for (int j = 0; j < kineTargets.Count; j++)
                        {
                            if (kineTargets[j] is CartesianTarget target && target.Motion == Motions.Linear)
                            {
                                target.Configuration = prevTarget.ProgramTargets[j].Kinematics.Configuration;
                            }
                        }

                        var kinematics = robotSystem.Kinematics(kineTargets, prevJoints);
                        cellTarget.SetTargetKinematics(kinematics, program.Errors, program.Warnings, prevTarget);
                        CheckUndefined(cellTarget, cellTargets);
                    }

                    double divisions = 1;
                    var linearTargets = cellTarget.ProgramTargets.Where(x => !x.IsJointMotion);
                    //   if (linearTargets.Count() > 0) program.Errors.Clear();

                    foreach (var target in linearTargets)
                    {
                        var prevPlane = target.GetPrevPlane(prevTarget.ProgramTargets[target.Group]);
                        double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
                        double divisionsTemp = Ceiling(distance / stepSize);
                        if (divisionsTemp > divisions) divisions = divisionsTemp;
                    }

                    var prevInterTarget = prevTarget.ShallowClone();
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
                        var interTarget = cellTarget.ShallowClone();
                        var kineTargets = cellTarget.Lerp(prevTarget, robotSystem, t, 0.0, 1.0);
                        var kinematics = program.RobotSystem.Kinematics(kineTargets, prevJoints);
                        interTarget.SetTargetKinematics(kinematics, program.Errors, null, prevInterTarget);

                        // Set speed

                        double slowestDelta = 0;
                        double slowestMinTime = 0;
                        foreach (var target in interTarget.ProgramTargets)
                        {
                            var speeds = GetSpeeds(target, prevInterTarget.ProgramTargets[target.Group]);
                            slowestDelta = Max(slowestDelta, speeds.Item1);
                            slowestMinTime = Max(slowestMinTime, speeds.Item2);
                            target.LeadingJoint = speeds.Item3;
                        }

                        if ((j > 1) && (Abs(slowestDelta - lastDeltaTime) > 1E-09))
                        {
                            keyframes.Add(prevInterTarget.ShallowClone());
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

                    if (program.Errors.Count > 0) { errorIndex = i; }

                    keyframes.Add(prevInterTarget.ShallowClone());

                    if (errorIndex == -1)
                    {
                        double longestWaitTime = 0;
                        foreach (var target in cellTarget.ProgramTargets)
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
                            keyframes.Add(prevInterTarget.ShallowClone());
                        }
                    }

                    // set target kinematics

                    cellTarget.TotalTime = time;
                    cellTarget.DeltaTime = totalDeltaTime;
                    cellTarget.MinTime = totalMinTime;

                    foreach (var programTarget in cellTarget.ProgramTargets)
                    {
                        int group = programTarget.Group;
                        programTarget.Kinematics = prevInterTarget.ProgramTargets[group].Kinematics;
                        programTarget.ChangesConfiguration = prevInterTarget.ProgramTargets[group].ChangesConfiguration;
                        programTarget.LeadingJoint = prevInterTarget.ProgramTargets[group].LeadingJoint;
                    }
                }

                program.Duration = time;

                if (errorIndex != -1) return errorIndex;
            }

            return -1;
        }

        void CheckUndefined(CellTarget cellTarget, List<CellTarget> cellTargets)
        {
            if (cellTarget.Index < cellTargets.Count - 1)
            {
                int i = cellTarget.Index;
                foreach (var target in cellTarget.ProgramTargets.Where(x => x.Kinematics.Configuration == RobotConfigurations.Undefined))
                {
                    if (!cellTargets[i + 1].ProgramTargets[target.Group].IsJointMotion)
                    {
                        program.Errors.Add($"Undefined configuration (probably due to a singularity) in target {target.Index} of robot {target.Group} before a linear motion");
                        indexError = i;
                    }
                }
            }
        }

        void SetAttributeName(TargetAttribute attribute, IEnumerable<ProgramTarget> targets, string name)
        {
            var namedAttribute = attribute.CloneWithName<TargetAttribute>(name);

            int index = program.Attributes.FindIndex(x => x == attribute);
            program.Attributes[index] = namedAttribute;

            if (namedAttribute is Tool)
            {
                foreach (var target in targets) if (target.Target.Tool == attribute as Tool) { target.Target = target.Target.ShallowClone(); target.Target.Tool = namedAttribute as Tool; }
            }
            else if (namedAttribute is Frame)
            {
                foreach (var target in targets) if (target.Target.Frame == attribute as Frame) { target.Target = target.Target.ShallowClone(); target.Target.Frame = namedAttribute as Frame; }
            }
            else if (namedAttribute is Speed)
            {
                foreach (var target in targets) if (target.Target.Speed == attribute as Speed) { target.Target = target.Target.ShallowClone(); target.Target.Speed = namedAttribute as Speed; }
            }
            else if (namedAttribute is Zone)
            {
                foreach (var target in targets) if (target.Target.Zone == attribute as Zone) { target.Target = target.Target.ShallowClone(); target.Target.Zone = namedAttribute as Zone; }
            }
            else if (namedAttribute is Command)
            {
                for (int i = 0; i < program.InitCommands.Count; i++)
                    if (program.InitCommands[i] == attribute as Command) program.InitCommands[i] = namedAttribute as Command;

                foreach (var target in targets)
                {
                    var group = target.Commands;
                    for (int i = 0; i < group.Count; i++)
                        if (group[i] == attribute as Command) group[i] = namedAttribute as Command;
                }
            }
        }

        (double, double, int) GetSpeeds(ProgramTarget target, ProgramTarget prevTarget)
        {
            Plane prevPlane = target.GetPrevPlane(prevTarget);
            var joints = robotSystem.GetJoints(target.Group).ToArray();
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

            for (int i = 0; i < target.Target.External.Length; i++)
            {
                var joint = joints[i + 6];
                double jointSpeed = joint.MaxSpeed;
                if (joint is PrismaticJoint) jointSpeed = Min(jointSpeed, target.Target.Speed.TranslationExternal);
                else if (joint is RevoluteJoint) jointSpeed = Min(jointSpeed, target.Target.Speed.RotationExternal);

                double deltaCurrentExternalTime = Abs(target.Kinematics.Joints[i + 6] - prevTarget.Kinematics.Joints[i + 6]) / jointSpeed;

                if (deltaCurrentExternalTime > deltaExternalTime)
                {
                    deltaExternalTime = deltaCurrentExternalTime;
                    externalLeadingJoint = i + 6;
                }
            }

            if (target.Target.Speed.Time == 0)
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
                double[] deltaTimes = new double[] { deltaLinearTime, deltaRotationTime, deltaAxisTime, deltaExternalTime };
                int deltaIndex = -1;

                for (int i = 0; i < deltaTimes.Length; i++)
                {
                    if (deltaTimes[i] > deltaTime)
                    {
                        deltaTime = deltaTimes[i];
                        deltaIndex = i;
                    }
                }

                {
                    if (deltaTime < TimeTol)
                    {
                        program.Warnings.Add($"Position and orientation don't change for {target.Index}");
                    }
                    else if (deltaIndex == 1)
                    {
                        if (target.Index != lastIndex) program.Warnings.Add($"Rotation speed limit reached in target {target.Index}");
                        lastIndex = target.Index;
                    }
                    else if (deltaIndex == 2)
                    {
                        if (target.Index != lastIndex) program.Warnings.Add($"Axis {leadingJoint + 1} speed limit reached in target {target.Index}");
                        lastIndex = target.Index;
                    }
                    else if (deltaIndex == 3)
                    {
                        if (target.Index != lastIndex) program.Warnings.Add($"External axis {externalLeadingJoint + 1} speed limit reached in target {target.Index}");
                        leadingJoint = externalLeadingJoint;
                        lastIndex = target.Index;
                    }
                }
            }
            else
            {
                // Get slowest by time
                double deltaTimeTime = target.Target.Speed.Time;
                double[] deltaTimes = new double[] { deltaTimeTime, deltaAxisTime, deltaExternalTime };
                //int deltaIndex = -1;

                for (int i = 0; i < deltaTimes.Length; i++)
                {
                    if (deltaTimes[i] > deltaTime)
                    {
                        deltaTime = deltaTimes[i];
                        //deltaIndex = i;
                    }
                }
            }

            return (deltaTime, deltaAxisTime, leadingJoint);
        }
    }
}
