const io = require("socket.io")("192.168.10.253:7777", {
  cors: {
    origin: "*",
  },
});

console.log("Server is running on port 7777 🚀");

io.on("connection", (socket) => {
  console.log(`Connection from ${socket.id}`);
  socket.emit("message", "Hello, world!");
});
