using System.Linq.Expressions;
using System.Diagnostics;
using Eto.Drawing;
using Eto.Forms;
using Rhino.Resources;
using static Robots.Grasshopper.LibrariesForm;
using EtoCommand = Eto.Forms.Command;

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
    }
}

class LibrariesForm : ComponentForm
{
    // static

    const string _helpUrl = "https://github.com/visose/Robots/wiki/Robot-libraries";

    public static Label NewLabel(Expression<Func<LibraryItem, string>> bindText, TextAlignment align = TextAlignment.Left, Font? font = null)
    {
        var label = new Label
        {
            TextAlignment = align
        };

        if (font is not null)
            label.Font = font;

        label.TextBinding.BindDataContext(bindText);
        return label;
    }

    public static string Icons(LibraryItem item)
    {
        if (item.IsLocal && item.IsDownloaded)
            return "❕📁";

        if (item.IsLocal)
            return "📁";

        if (item.IsDownloaded && item.IsUpdateAvailable)
            return "⬆✔";

        if (item.IsDownloaded)
            return "✔";

        if (item.IsOnline)
            return "💾";

        return "⚠";
    }

    static string Description(LibraryItem item)
    {
        if (item.IsLocal && item.IsDownloaded)
            return "❕📁 Local override, installed";

        if (item.IsLocal && item.IsOnline)
            return "📁 Local, available online";

        if (item.IsLocal)
            return "📁 Local";

        if (item.IsDownloaded && item.IsUpdateAvailable)
            return "⬆✔ Installed, update available";

        if (item.IsDownloaded && !item.IsOnline)
            return "✔⚠ Installed, online missing";

        if (item.IsDownloaded)
            return "✔ Installed";

        if (item.IsOnline)
            return "💾 Available online";

        return "⚠ Unknown error";
    }

    // instance

    readonly OnlineLibrary _library;
    readonly GridView _grid;
    readonly StackLayout _detailView;

    public LibrariesForm()
    {
        _library = new OnlineLibrary();

        Title = "Robot libraries";
        BackgroundColor = Colors.White;
        MinimumSize = new Size(600, 300);
        Content = new StackLayout
        {
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

    async Task UpdateAsync()
    {
        await _library.UpdateLibraryAsync();

        var values = _library.Libraries.Values;
        var ordered = values.OrderBy(i => i.Name);
        _grid.DataStore = null;
        _grid.DataStore = ordered;

        if (values.Any())
            _grid.SelectRow(0);
    }

    async Task DownloadAsync()
    {
        var item = (LibraryItem)_detailView.DataContext;

        var success = item switch
        {
            { IsUpdateAvailable: true } => await _library.TryDownloadLibraryAsync(item),
            { IsDownloaded: true } => _library.TryRemoveDownloadedLibrary(item),
            _ => throw new ArgumentException("Invalid action")
        };

        if (!success)
            MessageBox.Show(this, $"{ItemActions(item)} error on {item.Name}", MessageBoxType.Error);

        _detailView.UpdateBindings(BindingUpdateMode.Destination);
        _grid.ReloadData(_grid.SelectedRow);
    }

    string ItemActions(LibraryItem item) => item switch
    {
        { IsDownloaded: true, IsUpdateAvailable: true } => "Update",
        { IsUpdateAvailable: true } => "Install",
        { IsDownloaded: true } => "Delete",
        _ => ""
    };

    GridView Grid() => new()
    {
        Size = new Size(300, 300),
        Border = BorderType.None,
        GridLines = GridLines.Horizontal,
        ShowHeader = false,
        AllowMultipleSelection = false,
        Columns =
        {
            new GridColumn
            {
                DataCell = CustomCell.Create<LibraryCell>(),
                Expand = true
            }
        }
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
                    new StackLayoutItem(NewAsyncButton(UpdateAsync, label: "Refresh list", runOnce: true), true),
                    new LinkButton
                    {
                        Text = "Help",
                        Command = new EtoCommand((s, e) => Process.Start(_helpUrl))
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
            NewLabel(i => Description(i), font: EtoFonts.NormalFont),
            new StackLayoutItem(null, true),
            NewDetailButton()
        }
    };

    StackLayout NewDetailButton()
    {
        var detailButton = NewAsyncButton(DownloadAsync);
        var button = (Button)detailButton.Items[0].Control;
        button.TextBinding.BindDataContext((LibraryItem i) => ItemActions(i));
        button.BindDataContext(s => s.Visible, (LibraryItem i) => ItemActions(i) != "");
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
            Size = new Size(22, 22),
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

            await actionAsync();

            button.Enabled = true;
            spinner.Visible = false;
        }
    }
}