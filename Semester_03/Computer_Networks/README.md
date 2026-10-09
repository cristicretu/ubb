# Computer Networks

Semester 3 · Year 2 · Python, C, Cisco Packet Tracer

Socket programming (TCP and UDP clients and servers in Python and C) in the first half, then IP addressing, subnetting and static routing in Cisco Packet Tracer.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | First UDP client/server in Python |
| [Labs/Lab_02](Labs/Lab_02) | TCP client/server pairs; numbered files solve exercises (max of two numbers, max of an array, sum of digits) |
| [Labs/Lab_03](Labs/Lab_03) | "Hello, world" TCP server written in Python, C and Node.js (socket.io) |
| [Labs/Lab_04](Labs/Lab_04) | Concurrent TCP server that forks a process per client |
| [Labs/Lab_07](Labs/Lab_07) | First Packet Tracer topology |
| [Labs/Lab_09](Labs/Lab_09) | Notes on masks, network/broadcast addresses and splitting a network into subnets |
| [Labs/Lab_10](Labs/Lab_10) | Subnetting calculation plus Packet Tracer networks with static routing |
| [Labs/Lab_11](Labs/Lab_11) | Static routing in Packet Tracer |
| [Labs/Lab_12](Labs/Lab_12) | Packet Tracer topology |
| [Labs/PT_Practice](Labs/PT_Practice) | Packet Tracer practice topologies for the practical exam |
| [Labs/Practice](Labs/Practice) | Socket exam practice problems, one folder each (see below) |
| [Labs/test](Labs/test) | Practical test: "Confused Calculator" TCP server that sometimes answers wrong on purpose, with C client; `t2.txt`/`t3.txt` hold problem statements |

Problems in `Labs/Practice` (Python servers, mostly C clients):

| Folder | Problem |
| --- | --- |
| [auction](Labs/Practice/auction) | Auction house: bids over TCP, current price broadcast over UDP, ends after 3 s without bids |
| [chars](Labs/Practice/chars) | Client sends a string and a char, server returns every index of that char (C server and client) |
| [chat](Labs/Practice/chat), [lucaCHAT](Labs/Practice/lucaCHAT) | Multi-client TCP chat servers |
| [color_map](Labs/Practice/color_map), [terrain](Labs/Practice/terrain) | Clients explore a shared map until every cell is explored |
| [f1](Labs/Practice/f1) | F1 qualifying: checkpoints over UDP, leaderboard over TCP |
| [guess_game](Labs/Practice/guess_game) | Clients take turns guessing a random number |
| [jumbled_word](Labs/Practice/jumbled_word) | Server sends a jumbled word, clients guess it |
| [pi](Labs/Practice/pi) | Monte Carlo pi: clients send random points over UDP, server sends the approximation |
| [quiz](Labs/Practice/quiz) | Server sends arithmetic questions, clients answer |
| [stocks](Labs/Practice/stocks) | Stock market: clients hold stocks, server sends portfolio value and prices every second |
| [test](Labs/Practice/test) | Minimal C TCP server/client |

## How to run

Python:

```sh
python3 server.py   # in one terminal
python3 client.py   # in another
```

C:

```sh
gcc server.c -o server && ./server
gcc client.c -o client && ./client
```

Packet Tracer files (`.pkt`) open in Cisco Packet Tracer.

## Notes

- Many servers bind to a hardcoded LAN IP (`172.20.10.14`, `192.168.1.138`, `172.30.240.46`, ...). Change it to `0.0.0.0` or `127.0.0.1` (and the matching IP in the client) before running.
- The `fork()` server in Lab_04 and the C code use POSIX APIs, so run them on Linux/macOS or WSL.
- Files without an extension (`client`, `server`, `main`) are compiled macOS arm64 binaries. Recompile instead of running them.
