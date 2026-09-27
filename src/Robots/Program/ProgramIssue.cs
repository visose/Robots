using System.Runtime.CompilerServices;

[assembly: InternalsVisibleTo("Robots.Tests")]
[assembly: InternalsVisibleTo("Robots.Grasshopper")]

namespace Robots;

enum IssueLevel { Warning, Error }

enum IssueKind
{
    ProgramNameInvalid,
    ToolpathInvalid,
    TargetAxesInvalid,
    KinematicError,
    FrameCouplingInvalid,
    AttributeDefaulted,
    AttributeDuplicateName,
    PayloadExceeded,
    ConfigurationChanged,
    MotionWarning,
    UnsupportedPostProcessorFeature,
    CommandInvalid
}

readonly record struct ProgramIssue(
    IssueLevel Level,
    IssueKind Kind,
    string Message,
    int? TargetIndex = null,
    int? RobotGroup = null,
    string? Source = null);
