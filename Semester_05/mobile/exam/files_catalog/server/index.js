const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const cors = require('cors');
const bodyParser = require('body-parser');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

app.use(cors());
app.use(bodyParser.json());

// In-memory storage for files
const files = [];
let nextId = 1;

// Seed data with different locations
const seedData = [
  { name: 'report.pdf', status: 'shared', size: 250, location: 'Documents', usage: 0 },
  { name: 'photo.jpg', status: 'open', size: 1200, location: 'Pictures', usage: 0 },
  { name: 'draft.txt', status: 'draft', size: 5, location: 'Documents', usage: 0 },
  { name: 'video.mp4', status: 'open', size: 5000, location: 'Downloads', usage: 0 },
  { name: 'secret.txt', status: 'secret', size: 2, location: 'Documents', usage: 0 },
  { name: 'image.png', status: 'shared', size: 800, location: 'Pictures', usage: 0 },
  { name: 'music.mp3', status: 'open', size: 3500, location: 'Downloads', usage: 0 },
  { name: 'presentation.pptx', status: 'shared', size: 1500, location: 'Documents', usage: 0 },
];

// Initialize seed data
seedData.forEach(file => {
  files.push({
    id: nextId++,
    ...file
  });
});

console.log(`[Server] Initialized with ${files.length} seed files`);

// Broadcast to all connected WebSocket clients
const broadcast = (data) => {
  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(data));
    }
  });
};

// WebSocket connection handling
wss.on('connection', (ws) => {
  console.log('[WebSocket] New client connected');
  
  ws.on('close', () => {
    console.log('[WebSocket] Client disconnected');
  });
});

// Routes

// POST /file - Create a new file
app.post('/file', (req, res) => {
  const { name, status, size, location } = req.body;
  
  // Validate required fields
  if (!name || !status || size === undefined || !location) {
    console.log('[POST /file] Error: Missing required fields');
    return res.status(400).json({ error: 'Missing required fields: name, status, size, location' });
  }
  
  // Validate size is a positive integer
  const sizeInt = parseInt(size);
  if (isNaN(sizeInt) || sizeInt <= 0) {
    console.log('[POST /file] Error: Invalid size value');
    return res.status(400).json({ error: 'Size must be a positive integer' });
  }
  
  // Create new file
  const newFile = {
    id: nextId++,
    name: String(name),
    status: String(status),
    size: sizeInt,
    location: String(location),
    usage: 0
  };
  
  files.push(newFile);
  console.log(`[POST /file] Created file: ID=${newFile.id}, Name=${newFile.name}, Location=${newFile.location}`);
  
  // Broadcast the new file to all WebSocket clients
  broadcast(newFile);
  console.log(`[WebSocket] Broadcasted new file (ID: ${newFile.id}) to all clients`);
  
  res.json(newFile);
});

// GET /all - Get all files
app.get('/all', (req, res) => {
  console.log(`[GET /all] Returning ${files.length} files`);
  res.json(files);
});

// GET /locations - Get all unique locations
app.get('/locations', (req, res) => {
  const locations = [...new Set(files.map(file => file.location))];
  console.log(`[GET /locations] Returning ${locations.length} unique locations: ${locations.join(', ')}`);
  res.json(locations);
});

// GET /files/:location - Get files by location
app.get('/files/:location', (req, res) => {
  const location = req.params.location;
  const filteredFiles = files.filter(file => file.location === location);
  console.log(`[GET /files/${location}] Returning ${filteredFiles.length} files`);
  res.json(filteredFiles);
});

// DELETE /file/:id - Delete a file by ID
app.delete('/file/:id', (req, res) => {
  const id = parseInt(req.params.id);
  
  if (isNaN(id) || id <= 0) {
    console.log(`[DELETE /file/${req.params.id}] Error: Invalid ID`);
    return res.status(400).json({ error: 'Invalid file ID' });
  }
  
  const index = files.findIndex(file => file.id === id);
  
  if (index === -1) {
    console.log(`[DELETE /file/${id}] Error: File not found`);
    return res.status(404).json({ error: 'File not found' });
  }
  
  const deletedFile = files[index];
  files.splice(index, 1);
  console.log(`[DELETE /file/${id}] Deleted file: Name=${deletedFile.name}, Location=${deletedFile.location}`);
  
  res.json({ success: true, message: 'File deleted successfully', deletedFile });
});

// Start Server
const PORT = 3000;
server.listen(PORT, '0.0.0.0', () => {
  console.log(`[Server] Running on port ${PORT} (0.0.0.0)`);
});
