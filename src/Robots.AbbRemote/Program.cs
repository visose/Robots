using Robots.AbbRemote;

AbbRemoteServer server = new(new());
return await server.Run(Console.In, Console.Out);
