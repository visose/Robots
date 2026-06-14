using Robots.AbbRemote;

AbbRemoteServer server = new(new());
return await server.RunAsync(Console.In, Console.Out);
