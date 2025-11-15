using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace HttpDownloaderLab
{
    class Program
    {
        static void Main(string[] args)
        {
            var urls = new[]
            {
                "http://example.com/",
                "http://example.org/",
                "http://info.cern.ch/"
            };

            Console.WriteLine($"downloading {urls.Length} files...\n");

            Console.WriteLine("--- callbacks ---");
            var tasks1 = urls.Select((url, index) => 
                DownloadWithCallbacks(url, $"callback_{index}.html")).ToArray();
            
            Task.WaitAll(tasks1);
            Console.WriteLine("callbacks done\n");

            Console.WriteLine("--- continuewith ---");
            var tasks2 = urls.Select((url, index) => 
                DownloadWithContinueWith(url, $"continue_{index}.html")).ToArray();
            
            Task.WaitAll(tasks2);
            Console.WriteLine("continuewith done\n");

            Console.WriteLine("--- async/await ---");
            var tasks3 = urls.Select((url, index) => 
                DownloadFileAsync(url, $"async_{index}.html")).ToArray();
            
            Task.WaitAll(tasks3);
            Console.WriteLine("async/await done\n");

            Console.WriteLine("all done!");
        }

        // impl 3: async/await with task wrappers
        static async Task DownloadFileAsync(string url, string outputPath)
        {
            Socket socket = null;
            try
            {
                var uri = new Uri(url);
                var host = uri.Host;
                var path = uri.PathAndQuery;
                var port = uri.Port;

                Console.WriteLine($"[async] {url} starting...");

                socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

                var addresses = await Dns.GetHostAddressesAsync(host);
                var endpoint = new IPEndPoint(addresses[0], port);

                await ConnectAsync(socket, endpoint);

                string request = $"GET {path} HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n\r\n";
                await SendAsync(socket, Encoding.ASCII.GetBytes(request));

                string headers = await ReceiveHeadersAsync(socket);
                int contentLength = ParseContentLength(headers);

                Console.WriteLine($"[async] {url} got {contentLength} bytes");

                byte[] body = await ReceiveExactlyAsync(socket, contentLength);

                await File.WriteAllBytesAsync(outputPath, body);

                Console.WriteLine($"[async] {url} saved to {outputPath}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[async] {url} error: {ex.Message}");
            }
            finally
            {
                socket?.Dispose();
            }
        }

        // wrap BeginConnect/EndConnect into a task
        static Task ConnectAsync(Socket socket, EndPoint endpoint)
        {
            var tcs = new TaskCompletionSource<bool>();

            socket.BeginConnect(endpoint, iar =>
            {
                try
                {
                    socket.EndConnect(iar);
                    tcs.SetResult(true);
                }
                catch (Exception ex)
                {
                    tcs.SetException(ex);
                }
            }, null);

            return tcs.Task;
        }

        // wrap BeginSend/EndSend into a task
        static Task SendAsync(Socket socket, byte[] data)
        {
            var tcs = new TaskCompletionSource<int>();

            socket.BeginSend(data, 0, data.Length, SocketFlags.None, iar =>
            {
                try
                {
                    int sent = socket.EndSend(iar);
                    tcs.SetResult(sent);
                }
                catch (Exception ex)
                {
                    tcs.SetException(ex);
                }
            }, null);

            return tcs.Task;
        }

        // read headers byte by byte until \r\n\r\n
        static async Task<string> ReceiveHeadersAsync(Socket socket)
        {
            var buffer = new byte[1];
            var headers = new StringBuilder();

            while (true)
            {
                int count = await ReceiveAsync(socket, buffer, 0, 1);
                if (count == 0) break;

                headers.Append((char)buffer[0]);

                if (headers.Length >= 4 &&
                    headers[headers.Length - 4] == '\r' &&
                    headers[headers.Length - 3] == '\n' &&
                    headers[headers.Length - 2] == '\r' &&
                    headers[headers.Length - 1] == '\n')
                {
                    break;
                }
            }

            return headers.ToString();
        }

        // wrap BeginReceive/EndReceive into a task
        static Task<int> ReceiveAsync(Socket socket, byte[] buffer, int offset, int count)
        {
            var tcs = new TaskCompletionSource<int>();

            socket.BeginReceive(buffer, offset, count, SocketFlags.None, iar =>
            {
                try
                {
                    int received = socket.EndReceive(iar);
                    tcs.SetResult(received);
                }
                catch (Exception ex)
                {
                    tcs.SetException(ex);
                }
            }, null);

            return tcs.Task;
        }

        static async Task<byte[]> ReceiveExactlyAsync(Socket socket, int count)
        {
            var buffer = new byte[count];
            int totalReceived = 0;

            while (totalReceived < count)
            {
                int received = await ReceiveAsync(socket, buffer, totalReceived, count - totalReceived);
                if (received == 0) break;
                totalReceived += received;
            }

            return buffer;
        }

        static int ParseContentLength(string headers)
        {
            foreach (var line in headers.Split('\n'))
            {
                if (line.StartsWith("Content-Length:", StringComparison.OrdinalIgnoreCase))
                {
                    return int.Parse(line.Substring("Content-Length:".Length).Trim());
                }
            }
            throw new Exception("Content-Length header not found");
        }

        // impl 2: continuewith chaining (bonus)
        static Task DownloadWithContinueWith(string url, string outputPath)
        {
            var uri = new Uri(url);
            Socket socket = null;

            Console.WriteLine($"[continue] {url} starting...");

            return Dns.GetHostAddressesAsync(uri.Host)
                .ContinueWith(dnsTask =>
                {
                    if (dnsTask.IsFaulted)
                        throw dnsTask.Exception.InnerException;

                    socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                    var endpoint = new IPEndPoint(dnsTask.Result[0], uri.Port);
                    return ConnectAsync(socket, endpoint);
                }, TaskContinuationOptions.OnlyOnRanToCompletion).Unwrap()
                .ContinueWith(connectTask =>
                {
                    if (connectTask.IsFaulted)
                    {
                        socket?.Dispose();
                        throw connectTask.Exception.InnerException;
                    }

                    string request = $"GET {uri.PathAndQuery} HTTP/1.1\r\nHost: {uri.Host}\r\nConnection: close\r\n\r\n";
                    return SendAsync(socket, Encoding.ASCII.GetBytes(request));
                }, TaskContinuationOptions.OnlyOnRanToCompletion).Unwrap()
                .ContinueWith(sendTask =>
                {
                    if (sendTask.IsFaulted)
                    {
                        socket?.Dispose();
                        throw sendTask.Exception.InnerException;
                    }
                    return ReceiveHeadersAsync(socket);
                }, TaskContinuationOptions.OnlyOnRanToCompletion).Unwrap()
                .ContinueWith(headersTask =>
                {
                    if (headersTask.IsFaulted)
                    {
                        socket?.Dispose();
                        throw headersTask.Exception.InnerException;
                    }

                    int contentLength = ParseContentLength(headersTask.Result);
                    Console.WriteLine($"[continue] {url} got {contentLength} bytes");
                    return ReceiveExactlyAsync(socket, contentLength);
                }, TaskContinuationOptions.OnlyOnRanToCompletion).Unwrap()
                .ContinueWith(bodyTask =>
                {
                    if (bodyTask.IsFaulted)
                    {
                        socket?.Dispose();
                        Console.WriteLine($"[continue] {url} error: {bodyTask.Exception.InnerException.Message}");
                        throw bodyTask.Exception.InnerException;
                    }

                    socket?.Dispose();
                    return File.WriteAllBytesAsync(outputPath, bodyTask.Result);
                }, TaskContinuationOptions.OnlyOnRanToCompletion).Unwrap()
                .ContinueWith(saveTask =>
                {
                    if (saveTask.IsFaulted)
                    {
                        Console.WriteLine($"[continue] {url} error: {saveTask.Exception.InnerException.Message}");
                    }
                    else
                    {
                        Console.WriteLine($"[continue] {url} saved to {outputPath}");
                    }
                });
        }

        // impl 1: event-driven callbacks
        class DownloadState
        {
            public Socket Socket;
            public Uri Uri;
            public byte[] Buffer = new byte[4096];
            public StringBuilder Headers = new StringBuilder();
            public int ContentLength;
            public MemoryStream Body = new MemoryStream();
            public string OutputPath;
            public TaskCompletionSource<bool> CompletionSource;
        }

        static Task DownloadWithCallbacks(string url, string outputPath)
        {
            var uri = new Uri(url);
            var socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            var tcs = new TaskCompletionSource<bool>();
            var state = new DownloadState 
            { 
                Socket = socket, 
                Uri = uri, 
                OutputPath = outputPath,
                CompletionSource = tcs
            };

            Console.WriteLine($"[callback] {url} starting...");

            Dns.BeginGetHostAddresses(uri.Host, DnsCallback, state);

            return tcs.Task;
        }

        static void DnsCallback(IAsyncResult ar)
        {
            var state = (DownloadState)ar.AsyncState;
            try
            {
                var addresses = Dns.EndGetHostAddresses(ar);
                var endpoint = new IPEndPoint(addresses[0], state.Uri.Port);
                
                state.Socket.BeginConnect(endpoint, ConnectCallback, state);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[callback] {state.Uri} dns failed: {ex.Message}");
                state.Socket.Dispose();
                state.CompletionSource.SetException(ex);
            }
        }

        static void ConnectCallback(IAsyncResult ar)
        {
            var state = (DownloadState)ar.AsyncState;
            try
            {
                state.Socket.EndConnect(ar);
                
                string request = $"GET {state.Uri.PathAndQuery} HTTP/1.1\r\nHost: {state.Uri.Host}\r\nConnection: close\r\n\r\n";
                byte[] requestBytes = Encoding.ASCII.GetBytes(request);
                state.Socket.BeginSend(requestBytes, 0, requestBytes.Length, SocketFlags.None, SendCallback, state);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[callback] {state.Uri} connect failed: {ex.Message}");
                state.Socket.Dispose();
                state.CompletionSource.SetException(ex);
            }
        }

        static void SendCallback(IAsyncResult ar)
        {
            var state = (DownloadState)ar.AsyncState;
            try
            {
                state.Socket.EndSend(ar);
                
                state.Socket.BeginReceive(state.Buffer, 0, 1, SocketFlags.None, ReceiveHeaderCallback, state);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[callback] {state.Uri} send failed: {ex.Message}");
                state.Socket.Dispose();
                state.CompletionSource.SetException(ex);
            }
        }

        static void ReceiveHeaderCallback(IAsyncResult ar)
        {
            var state = (DownloadState)ar.AsyncState;
            try
            {
                int bytesRead = state.Socket.EndReceive(ar);
                if (bytesRead == 0) 
                { 
                    state.Socket.Dispose();
                    state.CompletionSource.SetException(new Exception("connection closed early"));
                    return;
                }

                state.Headers.Append((char)state.Buffer[0]);

                string headers = state.Headers.ToString();
                if (headers.Contains("\r\n\r\n"))
                {
                    state.ContentLength = ParseContentLength(headers);
                    Console.WriteLine($"[callback] {state.Uri} got {state.ContentLength} bytes");
                    
                    state.Socket.BeginReceive(state.Buffer, 0, Math.Min(state.Buffer.Length, state.ContentLength - (int)state.Body.Length), 
                        SocketFlags.None, ReceiveBodyCallback, state);
                }
                else
                {
                    state.Socket.BeginReceive(state.Buffer, 0, 1, SocketFlags.None, ReceiveHeaderCallback, state);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[callback] {state.Uri} receive headers failed: {ex.Message}");
                state.Socket.Dispose();
                state.CompletionSource.SetException(ex);
            }
        }

        static void ReceiveBodyCallback(IAsyncResult ar)
        {
            var state = (DownloadState)ar.AsyncState;
            try
            {
                int bytesRead = state.Socket.EndReceive(ar);
                if (bytesRead == 0) 
                { 
                    state.Socket.Dispose();
                    state.CompletionSource.SetException(new Exception("connection closed before all data received"));
                    return;
                }

                state.Body.Write(state.Buffer, 0, bytesRead);

                if (state.Body.Length >= state.ContentLength)
                {
                    File.WriteAllBytes(state.OutputPath, state.Body.ToArray());
                    Console.WriteLine($"[callback] {state.Uri} saved to {state.OutputPath}");
                    state.Socket.Dispose();
                    state.CompletionSource.SetResult(true);
                }
                else
                {
                    int remaining = state.ContentLength - (int)state.Body.Length;
                    state.Socket.BeginReceive(state.Buffer, 0, Math.Min(state.Buffer.Length, remaining), 
                        SocketFlags.None, ReceiveBodyCallback, state);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[callback] {state.Uri} receive body failed: {ex.Message}");
                state.Socket.Dispose();
                state.CompletionSource.SetException(ex);
            }
        }
    }
}