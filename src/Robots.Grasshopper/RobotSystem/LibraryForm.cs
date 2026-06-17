using System.Linq.Expressions;
using Eto.Drawing;
using Eto.Forms;
using Rhino.Resources;
using static Robots.Grasshopper.LibraryForm;

namespace Robots.Grasshopper;

class LibraryCell : StackLayout
{
    public LibraryCell()
    {
        Padding = 5;
        Spacing = 0;
        Orientation = Orientation.Horizontal;
        VerticalContentAlignment = VerticalAlignment.Center;

        Items.Add(new StackLayoutItem(NewLabel(i => i.Name), true));
        Items.Add(NewLabel(i => Icons(i), TextAlignment.Right));
        Width = 280;
    }
}

class LibraryForm : ComponentForm
{
    // static

    const string _helpUrl = "https://github.com/visose/Robots/wiki/Robot-libraries";

    internal static Label NewLabel(Expression<Func<LibraryItem, string>> bindText, TextAlignment align = TextAlignment.Left, Font? font = null)
    {
        var label = new Label
        {
            TextAlignment = align,
            Font = font ?? EtoFonts.NormalFont
        };

        _ = label.TextBinding.BindDataContext(bindText);
        return label;
    }

    public static string Icons(LibraryItem item) => item switch
    {
        { IsLocal: true, IsDownloaded: true } => "❕📁",
        { IsLocal: true } => "📁",
        { IsDownloaded: true, IsUpdateAvailable: true } => "⬆✔",
        { IsDownloaded: true } => "✔",
        { IsOnline: true } => "💾",
        _ => "⚠"
    };

    // instance

    readonly OnlineLibrary _library;
    readonly GridView _grid;
    readonly StackLayout _detailView;

    public LibraryForm(OnlineLibrary library)
    {
        _library = library;

        Title = "Robot libraries";
        MinimumSize = new(600, 300);
        Content = new StackLayout
        {
            Style = Rhino.UI.Panels.EtoPanelStyleName,
            Orientation = Orientation.Horizontal,
            Spacing = 20,
            Padding = 10,
            Items =
            {
                ListView(_grid = Grid()),
                new StackLayoutItem(_detailView = DetailView(), VerticalAlignment.Stretch, true)
            }
        };

        _grid.SelectedRowsChanged += (s, e) => _detailView.DataContext = _grid.SelectedItem;
    }

    async Task ChangeLocalPathAsync()
    {
        var settings = Settings.Load();
        SelectFolderDialog dialog = new()
        {
            Title = "Select folder for local robot libraries",
            Directory = settings.LocalLibraryPath,
        };

        if (dialog.ShowDialog(this) != DialogResult.Ok)
            return;

        Settings.Save(settings with { LocalLibraryPath = dialog.Directory });
        await RefreshAsync();
    }

    async Task RefreshAsync()
    {
        try
        {
            await _library.UpdateLibraryAsync();
        }
        catch (Exception e)
        {
            string rateLimit = e.Message.Contains(": 403")
                ? "\n\nIt is possible you reached your rate limit. Please wait one hour for the limit to reset."
                : "";

            _ = MessageBox.Show(this, $"Error refreshing the library list.{rateLimit}\n\n{e.Message}", MessageBoxType.Error);
            return;
        }

        var values = _library.Libraries.Values;
        var ordered = values.OrderBy(i => i.Name).ToList();

        _grid.DataStore = ordered;

        if (ordered.Count == 0)
            return;

        int index = _grid.SelectedItem is not LibraryItem selected
            ? 0 : ordered.FindIndex(i => selected.Name.Equals(i.Name, StringComparison.OrdinalIgnoreCase));
        index = Math.Max(index, 0);

        _grid.ScrollToRow(index);
        _grid.SelectRow(index);
    }

    async Task DownloadAsync()
    {
        if (_detailView.DataContext is not LibraryItem item)
            return;

        var action = ItemActions(item);

        try
        {
            switch (item)
            {
                case { IsUpdateAvailable: true }:
                    await _library.DownloadLibraryAsync(item);
                    break;
                case { IsDownloaded: true }:
                    _library.RemoveDownloadedLibrary(item);
                    break;
                default:
                    throw new InvalidOperationException("Invalid action.");
            }
        }
        catch (Exception e)
        {
            _ = MessageBox.Show(this, $"{action} error on {item.Name}.\n\n{e.Message}", MessageBoxType.Error);
        }

        _detailView.UpdateBindings(BindingUpdateMode.Destination);
        _grid.DataStore = _grid.DataStore;
    }

    static string ItemActions(LibraryItem item) => item switch
    {
        { IsDownloaded: true, IsUpdateAvailable: true } => "Update",
        { IsUpdateAvailable: true } => "Install",
        { IsDownloaded: true } => "Remove",
        _ => ""
    };

    static GridView Grid() => new()
    {
        Size = new(300, 300),
        Border = BorderType.None,
        GridLines = GridLines.Horizontal,
        ShowHeader = false,
        AllowMultipleSelection = false,
        Columns =
        {
            new GridColumn
            {
                DataCell = CustomCell.Create<LibraryCell>()
                // Expand = true; // Not available in early 7.0 releases.
            }
        },
        RowHeight = 31
    };

    StackLayout ListView(GridView grid) => new()
    {
        Spacing = 10,
        HorizontalContentAlignment = HorizontalAlignment.Stretch,
        Items =
        {
            new StackLayoutItem(new Scrollable
            {
                Border = BorderType.Line,
                ExpandContentWidth = true,
                ExpandContentHeight = false,
                Content = grid
            }, true),
            new StackLayout
            {
                Orientation = Orientation.Horizontal,
                VerticalContentAlignment = VerticalAlignment.Bottom,
                Items =
                {
                    new StackLayoutItem(NewAsyncButton(RefreshAsync, label: "Refresh", runOnce: true), true),
                    new StackLayoutItem(NewAsyncButton(ChangeLocalPathAsync, label: "Set local folder"), false),
                    new LinkButton
                    {
                        Text = "Help",
                        Command = new Eto.Forms.Command((s, e) => Application.Instance.Open(_helpUrl))
                    }
                }
            }
        }
    };

    StackLayout DetailView() => new()
    {
        Spacing = 10,
        Items =
        {
            NewLabel(i => i.Name, font: EtoFonts.BoldHeadingFont),
            NewLabel(i => Description(i)),
            new StackLayoutItem(null, true),
            NewDetailButton()
        }
    };

    static string Description(LibraryItem item) => item switch
    {
        { IsLocal: true, IsDownloaded: true } => "❕📁 Installed, local override",
        { IsLocal: true, IsOnline: true } => "📁 Local, available online",
        { IsLocal: true } => "📁 Local",
        { IsDownloaded: true, IsUpdateAvailable: true } => "⬆✔ Installed, update available",
        { IsDownloaded: true, IsOnline: false } => "✔⚠ Installed, online copy missing",
        { IsDownloaded: true } => "✔ Installed",
        { IsOnline: true } => "💾 Available online",
        _ => "⚠ Unknown error"
    };

    StackLayout NewDetailButton()
    {
        var detailButton = NewAsyncButton(DownloadAsync);
        var button = (Button)detailButton.Items[0].Control;
        _ = button.TextBinding.BindDataContext((LibraryItem i) => ItemActions(i));
        _ = button.BindDataContext(s => s.Visible, (LibraryItem i) => ItemActions(i) != "");
        return detailButton;
    }

    static StackLayout NewAsyncButton(Func<Task> actionAsync, string? label = null, bool runOnce = false)
    {
        Button button = new()
        {
            Text = label
        };

        Spinner spinner = new()
        {
            Visible = false
        };

        button.Click += async (s, e) => await ClickAsync();

        if (runOnce)
            button.PerformClick();

        return new StackLayout
        {
            Spacing = 5,
            Orientation = Orientation.Horizontal,
            VerticalContentAlignment = VerticalAlignment.Center,
            Items =
            {
                button,
                spinner
            }
        };

        async Task ClickAsync()
        {
            button.Enabled = false;
            spinner.Visible = true;
            spinner.Enabled = true;

            try
            {
                await actionAsync();
            }
            finally
            {
                button.Enabled = true;
                spinner.Visible = false;
                spinner.Enabled = false;
            }
        }
    }
}
