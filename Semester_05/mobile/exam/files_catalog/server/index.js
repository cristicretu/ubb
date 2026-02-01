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

const files = [];
let nextId = 1;

const seedData = [
  { name: 'report.pdf', status: 'shared', size: 250, location: 'Documents', usage: 45 },
  { name: 'photo.jpg', status: 'open', size: 1200, location: 'Pictures', usage: 120 },
  { name: 'draft.txt', status: 'draft', size: 5, location: 'Documents', usage: 8 },
  { name: 'video.mp4', status: 'open', size: 5000, location: 'Downloads', usage: 230 },
  { name: 'secret.txt', status: 'secret', size: 2, location: 'Documents', usage: 3 },
  { name: 'image.png', status: 'shared', size: 800, location: 'Pictures', usage: 67 },
  { name: 'music.mp3', status: 'open', size: 3500, location: 'Downloads', usage: 89 },
  { name: 'presentation.pptx', status: 'shared', size: 1500, location: 'Documents', usage: 156 },
];

seedData.forEach(file => files.push({ id: nextId++, ...file }));
console.log(`[Server] Initialized with ${files.length} files`);

const broadcast = (data) => {
  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(data));
    }
  });
};

wss.on('connection', (ws) => {
  console.log('[WS] Client connected');
  ws.on('close', () => console.log('[WS] Client disconnected'));
});

app.post('/file', (req, res) => {
  const { name, status, size, location } = req.body;
  
  if (!name || !status || size === undefined || !location) {
    return res.status(400).json({ error: 'Missing fields' });
  }
  
  const sizeInt = parseInt(size);
  if (isNaN(sizeInt) || sizeInt <= 0) {
    return res.status(400).json({ error: 'Invalid size' });
  }
  
  const newFile = { id: nextId++, name: String(name), status: String(status), size: sizeInt, location: String(location), usage: 0 };
  files.push(newFile);
  console.log(`[POST /file] ${newFile.name}`);
  
  broadcast(newFile);
  res.json(newFile);
});

app.get('/all', (req, res) => {
  console.log(`[GET /all] ${files.length} files`);
  res.json(files);
});

app.get('/locations', (req, res) => {
  const locations = [...new Set(files.map(f => f.location))];
  console.log(`[GET /locations] ${locations.join(', ')}`);
  res.json(locations);
});

app.get('/files/:location', (req, res) => {
  const filtered = files.filter(f => f.location === req.params.location);
  console.log(`[GET /files/${req.params.location}] ${filtered.length} files`);
  res.json(filtered);
});

app.delete('/file/:id', (req, res) => {
  const id = parseInt(req.params.id);
  if (isNaN(id) || id <= 0) return res.status(400).json({ error: 'Invalid ID' });
  
  const index = files.findIndex(f => f.id === id);
  if (index === -1) return res.status(404).json({ error: 'Not found' });
  
  const deleted = files.splice(index, 1)[0];
  console.log(`[DELETE /file/${id}] ${deleted.name}`);
  res.json({ success: true, deleted });
});

const PORT = 3001;
server.listen(PORT, '0.0.0.0', () => console.log(`[Server] Running on port ${PORT}`));
