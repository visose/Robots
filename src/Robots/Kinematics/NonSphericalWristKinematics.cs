using Rhino.Geometry;
using static System.Math;
using static Robots.GeometryMath;
using static Robots.Util;

namespace Robots;

readonly record struct WristSolution(
    double[] Joints,
    RobotConfigurations Configuration,
    bool IsNearSingular);

/// <summary>
/// Closed-form IK for the six-revolute standard-DH class with
/// parallel axes 2/3, intersecting axes 4/5, and an offset final wrist.
/// <see cref="Supports"/> defines the required geometry.
/// </summary>
class NonSphericalWristKinematics(RobotArm robot) : RobotKinematics(robot)
{
    const double EquationTolerance = 1e-8;
    const double BranchTolerance = 1e-7;
    const double SupportAngleTolerance = 1e-10;
    const double SupportDistanceTolerance = 1e-9;
    // Treat condition numbers of 100,000 or greater as near-singular.
    const double SingularRatioTolerance = 1e-5;
    const double RangeTolerance = 1e-10;
    const double StationaryTolerance = 1e-7;

    static readonly string[] NearSingularityErrors = ["Target near singularity."];

    readonly Joint[] _joints = robot.Joints;
    readonly double _maxReach = GetChainLength(robot.Joints);
    readonly double _geometryScale = GetGeometryScale(robot.Joints);
    readonly double _jacobianScale = GetJacobianScale(robot.Joints);

    public override bool CanSolve(RobotArm robot) => Supports(robot);

    public static bool Supports(RobotArm robot)
    {
        var joints = robot.Joints;
        ReadOnlySpan<double> alpha = [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];

        if (!HasRevoluteDh(joints, alpha, SupportAngleTolerance))
            return false;

        return Abs(joints[1].D) < SupportDistanceTolerance
            && Abs(joints[3].A) < SupportDistanceTolerance
            && Abs(joints[5].A) < SupportDistanceTolerance
            && Abs(joints[1].A) > SupportDistanceTolerance
            && Hypot(joints[2].A, joints[3].D) > SupportDistanceTolerance
            && Hypot(joints[4].A, joints[4].D) > SupportDistanceTolerance;
    }

    protected override InverseSolutions GetInverseSolutions(
        Transform transform,
        double[] external,
        PreviousJoints prevJoints,
        RobotConfigurations? requested)
    {
        var principal = GetPrincipalSolutions(transform, prevJoints, out var errors);
        var solutions = new List<InverseSolution>(principal.Count);

        if (requested is not null)
            AddPreferredSolutions(principal, prevJoints, requested, solutions);

        if (requested is null || solutions.Count == 0)
            AddPreferredSolutions(principal, prevJoints, requested: null, solutions);

        solutions.Sort(CompareSolutions);

        if (principal.Count > 0 && solutions.Count == 0)
            errors.Add("Target requires joints outside the permitted ranges.");

        int selected = SelectSolution(
            solutions,
            requested,
            prevJoints,
            preserveWindings: true,
            out _);

        if (selected >= 0 && IsNearSingular(solutions[selected].Joints))
            solutions[selected] = solutions[selected] with { Errors = NearSingularityErrors };

        return new(solutions, errors, PreserveWindings: true);
    }

    protected override bool TryGetConfiguration(
        double[] joints,
        out RobotConfigurations configuration)
    {
        configuration = RobotConfigurations.Undefined;

        if (joints.Length != _joints.Length)
            return false;

        for (int i = 0; i < joints.Length; i++)
        {
            var range = _joints[i].Range;

            if (!double.IsFinite(joints[i])
                || !(range.T0 - RangeTolerance < joints[i]
                    && joints[i] < range.T1 + RangeTolerance))
            {
                return false;
            }
        }

        double combinedAngle = joints[1] + joints[2];
        double signedRadius = _a[0]
            + _a[1] * Cos(joints[1])
            + _a[2] * Cos(combinedAngle)
            + _d[3] * Sin(combinedAngle);

        Span<RobotConfigurations> configurations = stackalloc RobotConfigurations[8];
        int count = GetConfigurations(
            joints,
            signedRadius,
            Abs(signedRadius) < BranchTolerance,
            configurations);

        configuration = configurations[0];

        for (int i = 1; i < count; i++)
        {
            if ((int)configurations[i] < (int)configuration)
                configuration = configurations[i];
        }

        return true;
    }

    public List<WristSolution> GetSolutions(
        Transform transform,
        double[]? previous,
        out List<string> errors) =>
        GetSolutions(transform, new PreviousJoints(previous), out errors);

    List<WristSolution> GetSolutions(
        Transform transform,
        PreviousJoints previous,
        out List<string> errors)
    {
        var principal = GetPrincipalSolutions(transform, previous, out errors);
        var solutions = new List<WristSolution>(principal.Count);
        double[]? classifiedJoints = null;
        bool isNearSingular = false;

        foreach (var candidate in principal)
        {
            if (!ReferenceEquals(candidate.Joints, classifiedJoints))
            {
                classifiedJoints = candidate.Joints;
                isNearSingular = IsNearSingular(candidate.Joints);
            }

            AddLiftedSolutions(candidate, isNearSingular, solutions);
        }

        solutions.Sort(CompareSolutions);

        if (principal.Count > 0 && solutions.Count == 0)
            errors.Add("Target requires joints outside the permitted ranges.");

        return solutions;
    }

    List<PrincipalSolution> GetPrincipalSolutions(
        Transform transform,
        PreviousJoints previous,
        out List<string> errors)
    {
        errors = [];

        double targetDistance = Sqrt(
            transform.M03 * transform.M03
            + transform.M13 * transform.M13
            + transform.M23 * transform.M23);

        if (targetDistance > _maxReach + DistanceTol)
        {
            errors.Add("Target out of reach.");
            return [];
        }

        double scale = GetScale(transform);
        var scaledA = new double[6];
        var scaledD = new double[6];

        for (int i = 0; i < 6; i++)
        {
            scaledA[i] = _a[i] / scale;
            scaledD[i] = _d[i] / scale;
        }

        var scaledTarget = transform;
        scaledTarget.M03 /= scale;
        scaledTarget.M13 /= scale;
        scaledTarget.M23 /= scale;
        var wrist = CreateWristContext(scaledA, scaledD, scaledTarget);

        var joint6Values = new List<double>(32);
        var tangentSeeds = new List<double>(WristPolynomial.MaxDegree);

        bool primaryCoverage = AddChartPair(
            scaledA,
            scaledD,
            scaledTarget,
            0,
            PI,
            joint6Values,
            tangentSeeds,
            out bool primaryIsStable);
        bool alternateCoverage = false;

        if (!primaryCoverage
            || !primaryIsStable)
        {
            alternateCoverage = AddChartPair(
                scaledA,
                scaledD,
                scaledTarget,
                HalfPI,
                -HalfPI,
                joint6Values,
                tangentSeeds,
                out _);
        }

        bool chartsCovered = primaryCoverage || alternateCoverage;

        if (!chartsCovered)
        {
            errors.Add("Analytical inverse kinematics could not resolve this target.");
            return [];
        }

        AddShoulderBoundaryRoots(scaledD[2], in wrist, previous, joint6Values);
        AddAxis4BoundaryRoots(scaledD[2], in wrist, previous, joint6Values, tangentSeeds);

        if (previous.HasValue)
            joint6Values.Add(NormalizeAngle(previous[5]));

        RefineJoint6Roots(scaledA, scaledD, in wrist, joint6Values, tangentSeeds);
        joint6Values.Sort();
        var principal = new List<PrincipalSolution>(32);

        foreach (double joint6 in joint6Values)
            BackSubstitute(scaledA, scaledD, in wrist, transform, joint6, previous, principal);

        if (principal.Count == 0)
            errors.Add("Target out of reach.");

        return principal;
    }

    static bool AddChartPair(
        double[] a,
        double[] d,
        Transform target,
        double firstPhase,
        double secondPhase,
        List<double> joint6Values,
        List<double> tangentSeeds,
        out bool isStable)
    {
        bool firstBuilt = AddChartRoots(
            a,
            d,
            target,
            firstPhase,
            joint6Values,
            tangentSeeds,
            out bool firstIsStable);
        bool secondBuilt = AddChartRoots(
            a,
            d,
            target,
            secondPhase,
            joint6Values,
            tangentSeeds,
            out bool secondIsStable);

        isStable = firstIsStable && secondIsStable;
        return firstBuilt && secondBuilt;
    }

    static bool AddChartRoots(
        double[] a,
        double[] d,
        Transform target,
        double phase,
        List<double> joint6Values,
        List<double> tangentSeeds,
        out bool isStable)
    {
        isStable = false;
        Span<double> coefficients = stackalloc double[WristPolynomial.MaxDegree + 1];

        if (!WristPolynomial.TryBuild(
            a,
            d,
            target,
            phase,
            coefficients,
            out var polynomial))
        {
            return false;
        }

        isStable = polynomial.IsStable;
        var polynomialCoefficients = coefficients[..(polynomial.Degree + 1)];

        Span<double> roots = stackalloc double[WristPolynomial.MaxDegree];
        Span<double> stationaryRoots = stackalloc double[WristPolynomial.MaxDegree - 1];

        if (PolynomialRoots.TryFind(
            polynomialCoefficients,
            roots,
            out int rootCount,
            stationaryRoots,
            out int stationaryRootCount))
        {
            AddJoint6Angles(roots[..rootCount], phase, joint6Values);
        }
        else
        {
            if (!ExactPolynomialRoots.TryFind(polynomialCoefficients, out var exactRoots))
                return false;

            AddJoint6Angles(exactRoots, phase, joint6Values);
            Span<double> derivative = stackalloc double[WristPolynomial.MaxDegree];
            int derivativeLength = Differentiate(polynomialCoefficients, derivative);

            if (!ExactPolynomialRoots.TryFind(
                derivative[..derivativeLength],
                out var exactStationaryRoots))
            {
                return false;
            }

            exactStationaryRoots.CopyTo(stationaryRoots);
            stationaryRootCount = exactStationaryRoots.Length;
        }

        foreach (double stationary in stationaryRoots[..stationaryRootCount])
        {
            if (Abs(Evaluate(polynomialCoefficients, stationary)) <= StationaryTolerance)
            {
                AddJoint6Angle(stationary, phase, joint6Values);
                AddJoint6Angle(stationary, phase, tangentSeeds);
            }
        }

        joint6Values.Add(phase);
        return true;
    }

    static int Differentiate(ReadOnlySpan<double> coefficients, Span<double> derivative)
    {
        if (coefficients.Length <= 1)
            return 0;

        for (int i = 1; i < coefficients.Length; i++)
            derivative[i - 1] = i * coefficients[i];

        return coefficients.Length - 1;
    }

    static double Evaluate(ReadOnlySpan<double> coefficients, double value)
    {
        double result = 0;

        for (int i = coefficients.Length - 1; i >= 0; i--)
            result = result * value + coefficients[i];

        return result;
    }

    static void AddShoulderBoundaryRoots(
        double shoulderOffset,
        in WristContext wrist,
        PreviousJoints previous,
        List<double> joint6Values)
    {
        if (Abs(shoulderOffset) >= EquationTolerance)
            return;

        double cosX = -wrist.A5 * wrist.Rx.X - wrist.D5 * wrist.Ry.X;
        double sinX = wrist.A5 * wrist.Ry.X - wrist.D5 * wrist.Rx.X;
        double cosY = -wrist.A5 * wrist.Rx.Y - wrist.D5 * wrist.Ry.Y;
        double sinY = wrist.A5 * wrist.Ry.Y - wrist.D5 * wrist.Rx.Y;
        double determinant = cosX * sinY - sinX * cosY;
        double coefficientScale = Max(1, Max(Hypot(cosX, sinX), Hypot(cosY, sinY)));

        if (Abs(determinant) > EquationTolerance * coefficientScale * coefficientScale)
        {
            double cos6 = (-wrist.P5.X * sinY + sinX * wrist.P5.Y) / determinant;
            double sin6 = (-cosX * wrist.P5.Y + wrist.P5.X * cosY) / determinant;
            double norm = Hypot(cos6, sin6);

            if (Abs(norm - 1) <= BranchTolerance)
                TryAddShoulderRoot(in wrist, Atan2(sin6 / norm, cos6 / norm), joint6Values);

            return;
        }

        double xLength = Hypot(cosX, sinX);
        double yLength = Hypot(cosY, sinY);

        if (xLength < EquationTolerance && yLength < EquationTolerance)
        {
            if (Hypot(wrist.P5.X, wrist.P5.Y) < EquationTolerance)
            {
                double preferred = previous.HasValue ? previous[5] : 0;
                TryAddShoulderRoot(in wrist, preferred, joint6Values);
            }

            return;
        }

        var roots = xLength >= yLength
            ? SolveLineCircle(cosX, sinX, -wrist.P5.X, out _)
            : SolveLineCircle(cosY, sinY, -wrist.P5.Y, out _);

        for (int i = 0; i < roots.Count; i++)
            TryAddShoulderRoot(in wrist, roots[i], joint6Values);
    }

    static void TryAddShoulderRoot(
        in WristContext wrist,
        double joint6,
        List<double> joint6Values)
    {
        GetWristGeometry(in wrist, joint6, out _, out _, out _, out var p4);

        if (p4.X * p4.X + p4.Y * p4.Y <= EquationTolerance * EquationTolerance)
            joint6Values.Add(NormalizeAngle(joint6));
    }

    static void AddAxis4BoundaryRoots(
        double shoulderOffset,
        in WristContext wrist,
        PreviousJoints previous,
        List<double> joint6Values,
        List<double> tangentSeeds)
    {
        double cosCoefficient = wrist.Ry.Z;
        double sinCoefficient = wrist.Rx.Z;
        var roots = SolveLineCircle(cosCoefficient, sinCoefficient, 0, out bool isContinuum);

        if (isContinuum)
        {
            double boundaryCos = wrist.P5.X * wrist.Ry.X
                + wrist.P5.Y * wrist.Ry.Y
                + wrist.P5.Z * wrist.Ry.Z;
            double boundarySin = wrist.P5.X * wrist.Rx.X
                + wrist.P5.Y * wrist.Rx.Y
                + wrist.P5.Z * wrist.Rx.Z;

            // With y5.Z identically zero, the axis-4 boundary reduces to
            // p5·y5 = d5 ± d3.
            for (int offsetSign = -1; offsetSign <= 1; offsetSign += 2)
            {
                var boundaryRoots = SolveLineCircle(
                    boundaryCos,
                    boundarySin,
                    wrist.D5 + offsetSign * shoulderOffset,
                    out bool boundaryContinuum);

                if (boundaryContinuum)
                {
                    double seed = previous.HasValue ? NormalizeAngle(previous[5]) : 0;
                    joint6Values.Add(seed);
                    tangentSeeds.Add(seed);
                    continue;
                }

                for (int rootIndex = 0; rootIndex < boundaryRoots.Count; rootIndex++)
                {
                    double root = boundaryRoots[rootIndex];
                    joint6Values.Add(root);
                    tangentSeeds.Add(root);
                }
            }

            return;
        }

        for (int i = 0; i < roots.Count; i++)
        {
            double root = roots[i];
            joint6Values.Add(root);
            tangentSeeds.Add(root);
        }
    }

    static void RefineJoint6Roots(
        double[] a,
        double[] d,
        in WristContext wrist,
        List<double> joint6Values,
        List<double> tangentSeeds)
    {
        int rootCount = joint6Values.Count;

        for (int rootIndex = 0; rootIndex < rootCount; rootIndex++)
        {
            double root = joint6Values[rootIndex];

            for (int radiusSign = -1; radiusSign <= 1; radiusSign += 2)
            {
                for (int combinedSign = -1; combinedSign <= 1; combinedSign += 2)
                {
                    if (TryRefineReachRoot(
                        a,
                        d,
                        in wrist,
                        root,
                        radiusSign,
                        combinedSign,
                        out double refined))
                    {
                        joint6Values.Add(refined);
                    }
                }
            }
        }

        foreach (double root in tangentSeeds)
        {
            for (int radiusSign = -1; radiusSign <= 1; radiusSign += 2)
            {
                for (int combinedSign = -1; combinedSign <= 1; combinedSign += 2)
                {
                    AddReachCrossings(
                        a,
                        d,
                        in wrist,
                        root,
                        radiusSign,
                        combinedSign,
                        joint6Values);

                    AddReachTangencies(
                        a,
                        d,
                        in wrist,
                        root,
                        radiusSign,
                        combinedSign,
                        joint6Values);
                }
            }
        }
    }

    static void AddReachCrossings(
        double[] a,
        double[] d,
        in WristContext wrist,
        double initial,
        int radiusSign,
        int combinedSign,
        List<double> joint6Values)
    {
        const double offset = 0.02;
        const int sideCount = 44;
        Span<ReachSample> samples = stackalloc ReachSample[sideCount * 2 + 1];
        int sampleCount = 0;

        for (int i = 0; i < sideCount; i++)
        {
            TryAddReachSample(
                a,
                d,
                in wrist,
                initial - ScaleB(offset, -i),
                radiusSign,
                combinedSign,
                samples,
                ref sampleCount);
        }

        TryAddReachSample(
            a,
            d,
            in wrist,
            initial,
            radiusSign,
            combinedSign,
            samples,
            ref sampleCount);

        for (int i = sideCount - 1; i >= 0; i--)
        {
            TryAddReachSample(
                a,
                d,
                in wrist,
                initial + ScaleB(offset, -i),
                radiusSign,
                combinedSign,
                samples,
                ref sampleCount);
        }

        for (int i = 1; i < sampleCount; i++)
        {
            var left = samples[i - 1];
            var right = samples[i];

            if (Abs(left.Residual) < 1e-13)
                joint6Values.Add(NormalizeAngle(left.Angle));

            if (Sign(left.Residual) == Sign(right.Residual))
                continue;

            for (int iteration = 0; iteration < 64; iteration++)
            {
                double midpoint = (left.Angle + right.Angle) * 0.5;

                if (!TryReachResidual(
                    a,
                    d,
                    in wrist,
                    midpoint,
                    radiusSign,
                    combinedSign,
                    out double residual))
                {
                    break;
                }

                if (Sign(left.Residual) == Sign(residual))
                    left = new(midpoint, residual);
                else
                    right = new(midpoint, residual);
            }

            double root = Abs(left.Residual) <= Abs(right.Residual) ? left.Angle : right.Angle;

            if (ReachError(a, d, in wrist, root, radiusSign, combinedSign) < 1e-10)
                joint6Values.Add(NormalizeAngle(root));
        }

        if (sampleCount > 0 && Abs(samples[sampleCount - 1].Residual) < 1e-13)
            joint6Values.Add(NormalizeAngle(samples[sampleCount - 1].Angle));
    }

    static void TryAddReachSample(
        double[] a,
        double[] d,
        in WristContext wrist,
        double angle,
        int radiusSign,
        int combinedSign,
        Span<ReachSample> samples,
        ref int sampleCount)
    {
        if (sampleCount > 0 && angle == samples[sampleCount - 1].Angle)
            return;

        if (TryReachResidual(
            a,
            d,
            in wrist,
            angle,
            radiusSign,
            combinedSign,
            out double residual))
        {
            samples[sampleCount++] = new(angle, residual);
        }
    }

    static void AddReachTangencies(
        double[] a,
        double[] d,
        in WristContext wrist,
        double initial,
        int radiusSign,
        int combinedSign,
        List<double> joint6Values)
    {
        var context = wrist;

        for (int extremumIndex = 0; extremumIndex < 2; extremumIndex++)
        {
            bool minimize = extremumIndex == 0;

            if (!TryFindReachExtremum(
                a,
                d,
                in context,
                initial,
                radiusSign,
                combinedSign,
                minimize,
                out double extremum,
                out double extremumResidual))
            {
                continue;
            }

            if (Abs(extremumResidual) < 1e-10)
                joint6Values.Add(NormalizeAngle(extremum));

            AddTangencySide(-1);
            AddTangencySide(1);

            void AddTangencySide(int direction)
            {
                double nearAngle = extremum;
                double nearResidual = extremumResidual;
                double distance = 1e-13;

                for (int i = 0; i < 40 && distance <= 0.02; i++)
                {
                    double farAngle = extremum + direction * distance;

                    if (!TryReachResidual(
                        a,
                        d,
                        in context,
                        farAngle,
                        radiusSign,
                        combinedSign,
                        out double farResidual))
                    {
                        distance *= 2;
                        continue;
                    }

                    if (Sign(nearResidual) != Sign(farResidual))
                    {
                        double root = BisectReachRoot(
                            a,
                            d,
                            in context,
                            nearAngle,
                            nearResidual,
                            farAngle,
                            farResidual,
                            radiusSign,
                            combinedSign);
                        joint6Values.Add(NormalizeAngle(root));
                        return;
                    }

                    nearAngle = farAngle;
                    nearResidual = farResidual;
                    distance *= 2;
                }
            }
        }
    }

    static bool TryFindReachExtremum(
        double[] a,
        double[] d,
        in WristContext wrist,
        double initial,
        int radiusSign,
        int combinedSign,
        bool minimize,
        out double extremum,
        out double residual)
    {
        var context = wrist;
        double left = initial - 0.02;
        double right = initial + 0.02;
        double first = right - (right - left) * 0.6180339887498949;
        double second = left + (right - left) * 0.6180339887498949;
        double firstValue = ReachObjective(first);
        double secondValue = ReachObjective(second);

        for (int iteration = 0; iteration < 96; iteration++)
        {
            if (firstValue <= secondValue)
            {
                right = second;
                second = first;
                secondValue = firstValue;
                first = right - (right - left) * 0.6180339887498949;
                firstValue = ReachObjective(first);
            }
            else
            {
                left = first;
                first = second;
                firstValue = secondValue;
                second = left + (right - left) * 0.6180339887498949;
                secondValue = ReachObjective(second);
            }
        }

        extremum = (left + right) * 0.5;
        return TryReachResidual(
            a,
            d,
            in context,
            extremum,
            radiusSign,
            combinedSign,
            out residual);

        double ReachObjective(double angle)
        {
            if (!TryReachResidual(
                a,
                d,
                in context,
                angle,
                radiusSign,
                combinedSign,
                out double value))
            {
                return double.PositiveInfinity;
            }

            return minimize ? value : -value;
        }
    }

    static double BisectReachRoot(
        double[] a,
        double[] d,
        in WristContext wrist,
        double firstAngle,
        double firstResidual,
        double secondAngle,
        double secondResidual,
        int radiusSign,
        int combinedSign)
    {
        if (firstAngle > secondAngle)
        {
            (firstAngle, secondAngle) = (secondAngle, firstAngle);
            (firstResidual, secondResidual) = (secondResidual, firstResidual);
        }

        for (int iteration = 0; iteration < 64; iteration++)
        {
            double midpoint = (firstAngle + secondAngle) * 0.5;

            if (!TryReachResidual(
                a,
                d,
                in wrist,
                midpoint,
                radiusSign,
                combinedSign,
                out double residual))
            {
                break;
            }

            if (Sign(firstResidual) == Sign(residual))
            {
                firstAngle = midpoint;
                firstResidual = residual;
            }
            else
            {
                secondAngle = midpoint;
                secondResidual = residual;
            }
        }

        return Abs(firstResidual) <= Abs(secondResidual) ? firstAngle : secondAngle;
    }

    static bool TryRefineReachRoot(
        double[] a,
        double[] d,
        in WristContext wrist,
        double initial,
        int radiusSign,
        int combinedSign,
        out double refined)
    {
        double current = initial;

        for (int iteration = 0; iteration < 16; iteration++)
        {
            if (!TryReachResidual(
                a,
                d,
                in wrist,
                current,
                radiusSign,
                combinedSign,
                out double residual))
            {
                break;
            }

            if (Abs(residual) < 1e-13)
            {
                refined = NormalizeAngle(current);
                return true;
            }

            const double step = 1e-5;

            if (!TryReachResidual(
                a,
                d,
                in wrist,
                current + step,
                radiusSign,
                combinedSign,
                out double after)
                || !TryReachResidual(
                    a,
                    d,
                    in wrist,
                    current - step,
                    radiusSign,
                    combinedSign,
                    out double before))
            {
                break;
            }

            double derivative = (after - before) / (2 * step);

            if (Abs(derivative) < 1e-12)
                break;

            double correction = Clamp(residual / derivative, -0.1, 0.1);
            current = NormalizeAngle(current - correction);

            if (Abs(correction) < 1e-13)
                break;
        }

        refined = NormalizeAngle(current);
        bool isRefined = TryReachResidual(
            a,
            d,
            in wrist,
            refined,
            radiusSign,
            combinedSign,
            out double finalResidual)
            && Abs(finalResidual) < 1e-10;

        return isRefined || TryMinimizeReachError(
            a,
            d,
            in wrist,
            initial,
            radiusSign,
            combinedSign,
            out refined);
    }

    static bool TryMinimizeReachError(
        double[] a,
        double[] d,
        in WristContext wrist,
        double initial,
        int radiusSign,
        int combinedSign,
        out double refined)
    {
        const int sampleCount = 32;
        const double halfWidth = 0.02;
        double left = initial - halfWidth;
        double step = 2 * halfWidth / sampleCount;
        int bestIndex = -1;
        double bestAngle = initial;
        double bestResidual = double.PositiveInfinity;

        for (int i = 0; i <= sampleCount; i++)
        {
            double angle = left + i * step;
            double residual = ReachError(a, d, in wrist, angle, radiusSign, combinedSign);

            if (residual < bestResidual)
            {
                bestIndex = i;
                bestAngle = angle;
                bestResidual = residual;
            }
        }

        if (bestIndex < 0)
        {
            refined = NormalizeAngle(initial);
            return false;
        }

        left += Max(0, bestIndex - 1) * step;
        double right = initial - halfWidth + Min(sampleCount, bestIndex + 1) * step;

        for (int iteration = 0; iteration < 64; iteration++)
        {
            double first = left + (right - left) / 3;
            double second = right - (right - left) / 3;
            double firstResidual = ReachError(a, d, in wrist, first, radiusSign, combinedSign);
            double secondResidual = ReachError(a, d, in wrist, second, radiusSign, combinedSign);

            if (firstResidual <= secondResidual)
                right = second;
            else
                left = first;
        }

        double candidate = (left + right) / 2;
        double candidateResidual = ReachError(a, d, in wrist, candidate, radiusSign, combinedSign);

        if (candidateResidual < bestResidual)
        {
            bestAngle = candidate;
            bestResidual = candidateResidual;
        }

        refined = NormalizeAngle(bestAngle);
        return bestResidual < 1e-10;
    }

    static double ReachError(
        double[] a,
        double[] d,
        in WristContext wrist,
        double joint6,
        int radiusSign,
        int combinedSign)
    {
        return TryReachResidual(
            a,
            d,
            in wrist,
            NormalizeAngle(joint6),
            radiusSign,
            combinedSign,
            out double residual)
            ? Abs(residual)
            : double.PositiveInfinity;
    }

    static bool TryReachResidual(
        double[] a,
        double[] d,
        in WristContext wrist,
        double joint6,
        int radiusSign,
        int combinedSign,
        out double residual)
    {
        GetWristGeometry(in wrist, joint6, out _, out var y5, out _, out var p4);
        double planarSquared = p4.X * p4.X + p4.Y * p4.Y;
        double signedRadiusSquared = planarSquared - d[2] * d[2];

        if (signedRadiusSquared < -EquationTolerance
            || planarSquared < EquationTolerance * EquationTolerance)
        {
            residual = 0;
            return false;
        }

        double signedRadius = radiusSign * Sqrt(Max(0, signedRadiusSquared));
        double cos1 = (signedRadius * p4.X - d[2] * p4.Y) / planarSquared;
        double sin1 = (d[2] * p4.X + signedRadius * p4.Y) / planarSquared;
        double norm1 = Hypot(cos1, sin1);

        if (norm1 < EquationTolerance)
        {
            residual = 0;
            return false;
        }

        cos1 /= norm1;
        sin1 /= norm1;
        double h = signedRadius - a[0];
        double height = p4.Z - d[0];
        double armCos = h * a[2] - height * d[3];
        double armSin = h * d[3] + height * a[2];
        double reach = (h * h + height * height + a[2] * a[2] + d[3] * d[3] - a[1] * a[1]) / 2;
        double horizontal = cos1 * y5.X + sin1 * y5.Y;
        double vertical = y5.Z;
        double orientationNorm = Hypot(horizontal, vertical);

        if (orientationNorm < EquationTolerance)
        {
            residual = 0;
            return false;
        }

        double cosCombined = combinedSign * horizontal / orientationNorm;
        double sinCombined = combinedSign * vertical / orientationNorm;
        residual = armCos * cosCombined + armSin * sinCombined - reach;
        return double.IsFinite(residual);
    }

    static void GetWristGeometry(
        in WristContext wrist,
        double joint6,
        out Vector3d x5,
        out Vector3d y5,
        out Vector3d rz,
        out Point3d p4)
    {
        double cos6 = Cos(joint6);
        double sin6 = Sin(joint6);
        rz = wrist.Rz;
        x5 = cos6 * wrist.Rx - sin6 * wrist.Ry;
        y5 = sin6 * wrist.Rx + cos6 * wrist.Ry;
        p4 = wrist.P5 - wrist.A5 * x5 - wrist.D5 * y5;
    }

    static WristContext CreateWristContext(
        double[] a,
        double[] d,
        Transform target)
    {
        var rx = new Vector3d(target.M00, target.M10, target.M20);
        var ry = new Vector3d(target.M01, target.M11, target.M21);
        var rz = new Vector3d(target.M02, target.M12, target.M22);
        var p5 = new Point3d(target.M03, target.M13, target.M23) - d[5] * rz;
        return new(rx, ry, rz, p5, a[4], d[4]);
    }

    static void AddJoint6Angles(
        ReadOnlySpan<double> roots,
        double phase,
        List<double> joint6Values)
    {
        foreach (double root in roots)
            AddJoint6Angle(root, phase, joint6Values);
    }

    static void AddJoint6Angle(double root, double phase, List<double> joint6Values) =>
        joint6Values.Add(NormalizeAngle(phase + 2 * Atan(root)));

    void BackSubstitute(
        double[] a,
        double[] d,
        in WristContext wrist,
        Transform unscaledTarget,
        double joint6,
        PreviousJoints previous,
        List<PrincipalSolution> solutions)
    {
        GetWristGeometry(in wrist, joint6, out var x5, out var y5, out var rz, out var p4);
        double planarSquared = p4.X * p4.X + p4.Y * p4.Y;
        double signedRadiusSquared = planarSquared - d[2] * d[2];

        if (signedRadiusSquared < -EquationTolerance)
            return;

        if (planarSquared < EquationTolerance * EquationTolerance && Abs(d[2]) < EquationTolerance)
        {
            SolveShoulderBoundary(
                a,
                d,
                unscaledTarget,
                p4,
                x5,
                y5,
                rz,
                joint6,
                previous,
                solutions);
            return;
        }

        if (planarSquared < EquationTolerance * EquationTolerance)
            return;

        double radius = Sqrt(Max(0, signedRadiusSquared));
        Span<double> signedRadii = [radius, -radius];
        int radiusCount = radius < BranchTolerance ? 1 : 2;

        for (int i = 0; i < radiusCount; i++)
        {
            double signedRadius = signedRadii[i];
            double cos1 = (signedRadius * p4.X - d[2] * p4.Y) / planarSquared;
            double sin1 = (d[2] * p4.X + signedRadius * p4.Y) / planarSquared;
            double norm1 = Hypot(cos1, sin1);

            if (norm1 < EquationTolerance)
                continue;

            cos1 /= norm1;
            sin1 /= norm1;
            double joint1 = Atan2(sin1, cos1);
            double h = signedRadius - a[0];
            double height = p4.Z - d[0];
            double armCos = h * a[2] - height * d[3];
            double armSin = h * d[3] + height * a[2];
            double reach = (h * h + height * height + a[2] * a[2] + d[3] * d[3] - a[1] * a[1]) / 2;
            double horizontal = cos1 * y5.X + sin1 * y5.Y;
            double vertical = y5.Z;
            var combinedAngles = SolveCombinedAngle(
                armCos,
                armSin,
                reach,
                horizontal,
                vertical,
                previous.HasValue ? previous[1] + previous[2] : 0);

            for (int combinedIndex = 0; combinedIndex < combinedAngles.Count; combinedIndex++)
            {
                double combinedAngle = combinedAngles[combinedIndex];

                TryAddSolution(
                    a,
                    d,
                    unscaledTarget,
                    x5,
                    y5,
                    rz,
                    joint1,
                    signedRadius,
                    h,
                    height,
                    combinedAngle,
                    joint6,
                    ambiguousShoulder: radius < BranchTolerance,
                    solutions);
            }
        }
    }

    void SolveShoulderBoundary(
        double[] a,
        double[] d,
        Transform unscaledTarget,
        Point3d p4,
        Vector3d x5,
        Vector3d y5,
        Vector3d rz,
        double joint6,
        PreviousJoints previous,
        List<PrincipalSolution> solutions)
    {
        double h = -a[0];
        double height = p4.Z - d[0];
        double armCos = h * a[2] - height * d[3];
        double armSin = h * d[3] + height * a[2];
        double reach = (h * h + height * height + a[2] * a[2] + d[3] * d[3] - a[1] * a[1]) / 2;
        var combinedAngles = SolveLineCircle(armCos, armSin, reach, out bool armContinuum);

        if (armContinuum)
        {
            double previousAngle = previous.HasValue ? previous[1] + previous[2] : 0;
            combinedAngles = AngleSolutions.One(NormalizeAngle(previousAngle));
        }

        Span<double> joint1Values = stackalloc double[2];

        for (int combinedIndex = 0; combinedIndex < combinedAngles.Count; combinedIndex++)
        {
            double combinedAngle = combinedAngles[combinedIndex];
            double sinCombined = Sin(combinedAngle);
            double cosCombined = Cos(combinedAngle);
            int joint1Count = 0;
            bool shoulderContinuum = false;

            if (Abs(sinCombined) > EquationTolerance)
            {
                double neededHorizontal = y5.Z * cosCombined / sinCombined;
                double horizontalLength = Hypot(y5.X, y5.Y);

                if (horizontalLength < EquationTolerance)
                {
                    if (Abs(neededHorizontal) > EquationTolerance)
                        continue;

                    shoulderContinuum = true;
                }
                else
                {
                    double ratio = neededHorizontal / horizontalLength;

                    if (ratio is < (-1 - EquationTolerance) or > (1 + EquationTolerance))
                        continue;

                    ratio = Clamp(ratio, -1, 1);
                    double center = Atan2(y5.Y, y5.X);
                    double offset = Acos(ratio);
                    joint1Values[joint1Count++] = NormalizeAngle(center + offset);

                    if (offset > BranchTolerance)
                        joint1Values[joint1Count++] = NormalizeAngle(center - offset);
                }
            }
            else if (Abs(y5.Z * cosCombined) <= EquationTolerance)
            {
                shoulderContinuum = true;
            }
            else
            {
                continue;
            }

            if (shoulderContinuum)
                joint1Values[joint1Count++] = previous.HasValue ? NormalizeAngle(previous[0]) : 0;

            for (int joint1Index = 0; joint1Index < joint1Count; joint1Index++)
            {
                double joint1 = joint1Values[joint1Index];

                TryAddSolution(
                    a,
                    d,
                    unscaledTarget,
                    x5,
                    y5,
                    rz,
                    joint1,
                    signedRadius: 0,
                    h,
                    height,
                    combinedAngle,
                    joint6,
                    ambiguousShoulder: true,
                    solutions);
            }
        }
    }

    void TryAddSolution(
        double[] a,
        double[] d,
        Transform target,
        Vector3d x5,
        Vector3d y5,
        Vector3d rz,
        double joint1,
        double signedRadius,
        double h,
        double height,
        double combinedAngle,
        double joint6,
        bool ambiguousShoulder,
        List<PrincipalSolution> solutions)
    {
        double cosCombined = Cos(combinedAngle);
        double sinCombined = Sin(combinedAngle);
        double upperX = h - a[2] * cosCombined - d[3] * sinCombined;
        double upperY = height - a[2] * sinCombined + d[3] * cosCombined;
        double upperLength = Hypot(upperX, upperY);

        if (Abs(upperLength - Abs(a[1])) > EquationTolerance * Max(1, Abs(a[1])))
            return;

        double joint2 = Atan2(upperY / a[1], upperX / a[1]);
        double joint3 = NormalizeAngle(combinedAngle - joint2);
        double cos1 = Cos(joint1);
        double sin1 = Sin(joint1);
        var x3 = new Vector3d(cos1 * cosCombined, sin1 * cosCombined, sinCombined);
        var y3 = new Vector3d(sin1, -cos1, 0);
        var z3 = new Vector3d(cos1 * sinCombined, sin1 * sinCombined, -cosCombined);

        if (Abs(z3 * y5) > EquationTolerance)
            return;

        double joint4 = Atan2(-(x3 * y5), y3 * y5);
        double joint5 = Atan2(-(z3 * x5), z3 * rz);
        double[] joints =
        [
            NormalizeAngle(joint1),
            NormalizeAngle(joint2),
            joint3,
            NormalizeAngle(joint4),
            NormalizeAngle(joint5),
            NormalizeAngle(joint6)
        ];

        if (!MatchesTarget(joints, target))
            return;

        AddConfigurations(
            joints,
            signedRadius,
            ambiguousShoulder,
            solutions);
    }

    static AngleSolutions SolveCombinedAngle(
        double armCos,
        double armSin,
        double reach,
        double horizontal,
        double vertical,
        double preferred)
    {
        double orientationNorm = Hypot(horizontal, vertical);

        if (orientationNorm < BranchTolerance)
        {
            var angles = SolveLineCircle(armCos, armSin, reach, out bool isContinuum);
            return isContinuum
                ? AngleSolutions.One(NormalizeAngle(preferred))
                : angles;
        }

        double cos = horizontal / orientationNorm;
        double sin = vertical / orientationNorm;
        bool hasFirst = SatisfiesReach(armCos, armSin, reach, cos, sin);
        double first = Atan2(sin, cos);

        cos = -cos;
        sin = -sin;
        bool hasSecond = SatisfiesReach(armCos, armSin, reach, cos, sin);
        double second = Atan2(sin, cos);

        return hasFirst
            ? hasSecond ? AngleSolutions.Two(first, second) : AngleSolutions.One(first)
            : hasSecond ? AngleSolutions.One(second) : default;
    }

    static AngleSolutions SolveLineCircle(
        double armCos,
        double armSin,
        double reach,
        out bool isContinuum)
    {
        double length = Hypot(armCos, armSin);
        isContinuum = false;

        if (length < EquationTolerance)
        {
            if (Abs(reach) < EquationTolerance)
                isContinuum = true;

            return default;
        }

        double ratio = reach / length;

        if (ratio is < (-1 - EquationTolerance) or > (1 + EquationTolerance))
            return default;

        ratio = Clamp(ratio, -1, 1);
        double center = Atan2(armSin, armCos);
        double offset = Acos(ratio);

        return offset < BranchTolerance
            ? AngleSolutions.One(NormalizeAngle(center))
            : AngleSolutions.Two(
                NormalizeAngle(center + offset),
                NormalizeAngle(center - offset));
    }

    static bool SatisfiesReach(
        double armCos,
        double armSin,
        double reach,
        double cos,
        double sin)
    {
        double actual = armCos * cos + armSin * sin;
        double scale = Max(1, Max(Abs(actual), Abs(reach)));
        return Abs(actual - reach) <= EquationTolerance * scale;
    }

    void AddConfigurations(
        double[] joints,
        double signedRadius,
        bool ambiguousShoulder,
        List<PrincipalSolution> solutions)
    {
        Span<RobotConfigurations> configurations = stackalloc RobotConfigurations[8];
        int count = GetConfigurations(
            joints,
            signedRadius,
            ambiguousShoulder,
            configurations);

        for (int i = 0; i < count; i++)
        {
            var candidate = new PrincipalSolution(
                joints,
                configurations[i]);

            if (!Contains(solutions, candidate))
                solutions.Add(candidate);
        }
    }

    int GetConfigurations(
        double[] joints,
        double signedRadius,
        bool ambiguousShoulder,
        Span<RobotConfigurations> configurations)
    {
        bool nominalShoulder = signedRadius < 0;
        double bend = Sin(joints[2] - Atan2(_d[3], _a[2]));
        bool ambiguousElbow = Abs(bend) < BranchTolerance;
        bool physicalElbow = bend > 0;
        bool ambiguousWrist = Abs(Sin(joints[4])) < BranchTolerance;
        bool nominalWrist = NormalizeAngle(joints[4]) > 0;
        int shoulderCount = ambiguousShoulder ? 2 : 1;
        int wristCount = ambiguousWrist ? 2 : 1;
        int count = 0;

        for (int shoulderIndex = 0; shoulderIndex < shoulderCount; shoulderIndex++)
        {
            bool shoulder = shoulderIndex == 0 ? nominalShoulder : !nominalShoulder;
            bool nominalElbow = physicalElbow ^ shoulder;
            int elbowCount = ambiguousElbow ? 2 : 1;

            for (int elbowIndex = 0; elbowIndex < elbowCount; elbowIndex++)
            {
                bool elbow = elbowIndex == 0 ? nominalElbow : !nominalElbow;

                for (int wristIndex = 0; wristIndex < wristCount; wristIndex++)
                {
                    bool wrist = wristIndex == 0 ? nominalWrist : !nominalWrist;
                    RobotConfigurations configuration = RobotConfigurations.None;

                    if (shoulder)
                        configuration |= RobotConfigurations.Shoulder;

                    if (elbow)
                        configuration |= RobotConfigurations.Elbow;

                    if (wrist)
                        configuration |= RobotConfigurations.Wrist;

                    bool isDuplicate = false;

                    for (int i = 0; i < count; i++)
                    {
                        if (configurations[i] == configuration)
                        {
                            isDuplicate = true;
                            break;
                        }
                    }

                    if (!isDuplicate)
                        configurations[count++] = configuration;
                }
            }
        }

        return count;
    }

    bool MatchesTarget(double[] joints, Transform target)
    {
        Span<Transform> transforms = stackalloc Transform[6];
        DH(joints, transforms);
        var actual = transforms[^1];
        double dx = actual.M03 - target.M03;
        double dy = actual.M13 - target.M13;
        double dz = actual.M23 - target.M23;
        double positionError = Sqrt(dx * dx + dy * dy + dz * dz);
        double positionTolerance = Max(1e-5, GetScale(target) * 1e-9);
        double orientationError = 0;

        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 3; column++)
                orientationError = Max(orientationError, Abs(actual[row, column] - target[row, column]));
        }

        return positionError <= positionTolerance && orientationError <= 1e-8;
    }

    bool IsNearSingular(double[] joints)
    {
        Span<Transform> transforms = stackalloc Transform[6];
        DH(joints, transforms);
        return IsNearSingular(transforms);
    }

    bool IsNearSingular(ReadOnlySpan<Transform> transforms)
    {
        Span<double> jacobian = stackalloc double[36];
        var end = transforms[^1];
        double endX = end.M03;
        double endY = end.M13;
        double endZ = end.M23;
        double inverseScale = 1 / _jacobianScale;

        for (int column = 0; column < 6; column++)
        {
            double originX;
            double originY;
            double originZ;
            double axisX;
            double axisY;
            double axisZ;

            if (column == 0)
            {
                originX = 0;
                originY = 0;
                originZ = 0;
                axisX = 0;
                axisY = 0;
                axisZ = 1;
            }
            else
            {
                var previous = transforms[column - 1];
                originX = previous.M03;
                originY = previous.M13;
                originZ = previous.M23;
                axisX = previous.M02;
                axisY = previous.M12;
                axisZ = previous.M22;
            }

            double offsetX = endX - originX;
            double offsetY = endY - originY;
            double offsetZ = endZ - originZ;
            jacobian[column] = (axisY * offsetZ - axisZ * offsetY) * inverseScale;
            jacobian[6 + column] = (axisZ * offsetX - axisX * offsetZ) * inverseScale;
            jacobian[12 + column] = (axisX * offsetY - axisY * offsetX) * inverseScale;
            jacobian[18 + column] = axisX;
            jacobian[24 + column] = axisY;
            jacobian[30 + column] = axisZ;
        }

        return JacobianCondition.MinimumSingularRatio(jacobian)
            <= SingularRatioTolerance;
    }

    void AddLiftedSolutions(
        PrincipalSolution principal,
        bool isNearSingular,
        List<WristSolution> solutions)
    {
        var current = new double[principal.Joints.Length];
        AddJoint(0);

        void AddJoint(int index)
        {
            if (index == principal.Joints.Length)
            {
                var solution = new WristSolution(
                    [.. current],
                    principal.Configuration,
                    isNearSingular);

                if (!Contains(solutions, solution))
                    solutions.Add(solution);

                return;
            }

            double angle = NormalizeAngle(principal.Joints[index]);
            var range = _joints[index].Range;
            int minTurn = (int)Ceiling((range.T0 - angle - RangeTolerance) / PI2);
            int maxTurn = (int)Floor((range.T1 - angle + RangeTolerance) / PI2);

            for (int turn = minTurn; turn <= maxTurn; turn++)
            {
                current[index] = angle + turn * PI2;
                AddJoint(index + 1);
            }
        }
    }

    double GetScale(Transform transform)
    {
        return Max(
            _geometryScale,
            Max(Abs(transform.M03), Max(Abs(transform.M13), Abs(transform.M23))));
    }

    static double GetGeometryScale(Joint[] joints)
    {
        double scale = 1;

        foreach (var joint in joints)
            scale = Max(scale, Max(Abs(joint.A), Abs(joint.D)));

        return scale;
    }

    static double GetJacobianScale(Joint[] joints)
    {
        double scale = Abs(joints[0].A);

        for (int i = 1; i < joints.Length; i++)
            scale += Hypot(joints[i].A, joints[i].D);

        return scale;
    }

    int CompareSolutions(WristSolution first, WristSolution second) =>
        CompareSolutions(first.Configuration, first.Joints, second.Configuration, second.Joints);

    int CompareSolutions(InverseSolution first, InverseSolution second) =>
        CompareSolutions(first.Configuration, first.Joints, second.Configuration, second.Joints);

    int CompareSolutions(
        RobotConfigurations firstConfiguration,
        double[] firstJoints,
        RobotConfigurations secondConfiguration,
        double[] secondJoints)
    {
        int configuration = ((int)firstConfiguration).CompareTo((int)secondConfiguration);

        if (configuration != 0)
            return configuration;

        double firstDistance = MidpointDistance(firstJoints);
        double secondDistance = MidpointDistance(secondJoints);
        int distance = firstDistance.CompareTo(secondDistance);

        if (distance != 0)
            return distance;

        for (int i = 0; i < firstJoints.Length; i++)
        {
            int joint = firstJoints[i].CompareTo(secondJoints[i]);

            if (joint != 0)
                return joint;
        }

        return 0;
    }

    void AddPreferredSolutions(
        List<PrincipalSolution> principal,
        PreviousJoints previous,
        RobotConfigurations? requested,
        List<InverseSolution> solutions)
    {
        foreach (var candidate in principal)
        {
            if (requested is RobotConfigurations configuration
                && candidate.Configuration != configuration)
            {
                continue;
            }

            if (TryGetPreferredWinding(candidate.Joints, previous, out var joints))
                solutions.Add(new(joints, candidate.Configuration));
        }
    }

    bool TryGetPreferredWinding(
        double[] principal,
        PreviousJoints previous,
        out double[] joints)
    {
        joints = [];
        Span<double> values = stackalloc double[principal.Length];

        for (int i = 0; i < principal.Length; i++)
        {
            double angle = NormalizeAngle(principal[i]);
            var range = _joints[i].Range;
            int minTurn = (int)Ceiling((range.T0 - angle - RangeTolerance) / PI2);
            int maxTurn = (int)Floor((range.T1 - angle + RangeTolerance) / PI2);

            if (minTurn > maxTurn)
                return false;

            double midpoint = (range.T0 + range.T1) * 0.5;
            double preferred = previous.HasValue ? previous[i] : midpoint;
            double value = 0;
            double bestDifference = double.MaxValue;
            double bestMidpointDifference = double.MaxValue;

            for (int turn = minTurn; turn <= maxTurn; turn++)
            {
                double candidate = angle + turn * PI2;
                double difference = candidate - preferred;
                difference *= difference;
                double midpointDifference = candidate - midpoint;
                midpointDifference *= midpointDifference;

                if (difference < bestDifference
                    || (difference == bestDifference
                        && (midpointDifference < bestMidpointDifference
                            || (midpointDifference == bestMidpointDifference
                                && candidate < value))))
                {
                    value = candidate;
                    bestDifference = difference;
                    bestMidpointDifference = midpointDifference;
                }
            }

            values[i] = value;
        }

        joints = values.ToArray();
        return true;
    }

    double MidpointDistance(double[] joints)
    {
        double distance = 0;

        for (int i = 0; i < joints.Length; i++)
        {
            var range = _joints[i].Range;
            double difference = joints[i] - (range.T0 + range.T1) * 0.5;
            distance += difference * difference;
        }

        return distance;
    }

    static bool Contains(
        IReadOnlyList<WristSolution> solutions,
        WristSolution candidate)
    {
        foreach (var solution in solutions)
        {
            if (solution.Configuration == candidate.Configuration
                && SameLiftedJoints(solution.Joints, candidate.Joints))
            {
                return true;
            }
        }

        return false;
    }

    static bool Contains(
        IReadOnlyList<PrincipalSolution> solutions,
        PrincipalSolution candidate)
    {
        foreach (var solution in solutions)
        {
            if (solution.Configuration == candidate.Configuration
                && SamePrincipalJoints(solution.Joints, candidate.Joints))
            {
                return true;
            }
        }

        return false;
    }

    static bool SameLiftedJoints(double[] first, double[] second)
    {
        for (int i = 0; i < first.Length; i++)
        {
            if (Abs(first[i] - second[i]) > BranchTolerance)
                return false;
        }

        return true;
    }

    static bool SamePrincipalJoints(double[] first, double[] second)
    {
        for (int i = 0; i < first.Length; i++)
        {
            if (CircularDistance(first[i], second[i]) > RangeTolerance)
                return false;
        }

        return true;
    }

    static double CircularDistance(double first, double second) =>
        Abs(IEEERemainder(first - second, PI2));

    readonly record struct PrincipalSolution(
        double[] Joints,
        RobotConfigurations Configuration);

    readonly record struct WristContext(
        Vector3d Rx,
        Vector3d Ry,
        Vector3d Rz,
        Point3d P5,
        double A5,
        double D5);

    readonly record struct ReachSample(double Angle, double Residual);

    readonly record struct AngleSolutions(double First, double Second, int Count)
    {
        public double this[int index] => index switch
        {
            0 when Count > 0 => First,
            1 when Count > 1 => Second,
            _ => throw new ArgumentOutOfRangeException(nameof(index))
        };

        public static AngleSolutions One(double value) => new(value, 0, 1);

        public static AngleSolutions Two(double first, double second) => new(first, second, 2);
    }
}
