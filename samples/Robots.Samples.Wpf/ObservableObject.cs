using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace Robots.Samples.Wpf;

public abstract class ObservableObject : INotifyPropertyChanged
{
    public event PropertyChangedEventHandler? PropertyChanged;

    protected void OnPropertyChanged([CallerMemberName] string property = "")
    {
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(property));
    }

    protected void SetField<T>(ref T field, T value, [CallerMemberName] string property = "")
    {
        field = value;
        OnPropertyChanged(property);
    }
}
