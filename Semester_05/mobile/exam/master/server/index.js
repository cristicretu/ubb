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

const items = [];
let nextId = 1;

const seedData = [
  { name: 'Item Alpha', status: 'available', owner: '', value1: 500, value2: 12 },
  { name: 'Item Beta', status: 'pending', owner: 'Alice', value1: 1200, value2: 28 },
  { name: 'Item Gamma', status: 'available', owner: '', value1: 320, value2: 5 },
  { name: 'Item Delta', status: 'done', owner: 'Bob', value1: 1800, value2: 35 },
  { name: 'Item Epsilon', status: 'available', owner: '', value1: 650, value2: 8 },
  { name: 'Item Zeta', status: 'pending', owner: 'Carol', value1: 980, value2: 42 },
  { name: 'Item Eta', status: 'canceled', owner: '', value1: 750, value2: 3 },
  { name: 'Item Theta', status: 'available', owner: '', value1: 1100, value2: 19 },
  { name: 'Item Iota', status: 'done', owner: 'David', value1: 420, value2: 22 },
  { name: 'Item Kappa', status: 'available', owner: '', value1: 550, value2: 15 },
  { name: 'Item Lambda', status: 'pending', owner: 'Alice', value1: 1350, value2: 50 },
  { name: 'Item Mu', status: 'canceled', owner: '', value1: 280, value2: 7 },
  { name: 'Item Nu', status: 'available', owner: '', value1: 890, value2: 31 },
  { name: 'Item Xi', status: 'done', owner: 'Bob', value1: 1650, value2: 18 },
  { name: 'Item Omicron', status: 'available', owner: '', value1: 720, value2: 11 },
];

seedData.forEach(item => items.push({ id: nextId++, ...item }));
console.log(`[Server] Initialized with ${items.length} items`);

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

app.post('/item', (req, res) => {
  const { name, status, owner, value1 } = req.body;

  if (!name || !status || value1 === undefined) {
    console.log('[POST /item] Error: Missing required fields');
    return res.status(400).json({ error: 'Missing required fields: name, status, value1' });
  }

  const value1Int = parseInt(value1);
  if (isNaN(value1Int) || value1Int < 0) {
    console.log('[POST /item] Error: Invalid value1');
    return res.status(400).json({ error: 'Invalid value1: must be a non-negative integer' });
  }

  const newItem = {
    id: nextId++,
    name: String(name),
    status: String(status),
    owner: owner ? String(owner) : '',
    value1: value1Int,
    value2: 0
  };

  items.push(newItem);
  console.log(`[POST /item] Created: ${newItem.name} (ID: ${newItem.id})`);

  broadcast(newItem);
  res.json(newItem);
});

app.get('/all', (req, res) => {
  console.log(`[GET /all] Returning ${items.length} items`);
  res.json(items);
});

app.get('/items/:owner', (req, res) => {
  const ownerParam = req.params.owner;
  const filtered = items.filter(item => 
    item.owner.toLowerCase() === ownerParam.toLowerCase()
  );
  console.log(`[GET /items/${ownerParam}] Found ${filtered.length} items`);
  res.json(filtered);
});

app.get('/available', (req, res) => {
  const available = items.filter(item => item.status === 'available');
  console.log(`[GET /available] Found ${available.length} items`);
  res.json(available);
});

app.delete('/item/:id', (req, res) => {
  const id = parseInt(req.params.id);

  if (isNaN(id) || id <= 0) {
    console.log(`[DELETE /item] Error: Invalid ID`);
    return res.status(400).json({ error: 'Invalid ID' });
  }

  const index = items.findIndex(item => item.id === id);
  if (index === -1) {
    console.log(`[DELETE /item/${id}] Error: Item not found`);
    return res.status(404).json({ error: 'Item not found' });
  }

  const deleted = items.splice(index, 1)[0];
  console.log(`[DELETE /item/${id}] Deleted: ${deleted.name}`);

  res.json({ success: true, deleted });
});

app.post('/action', (req, res) => {
  const { itemId, status, owner } = req.body;

  if (!itemId) {
    console.log('[POST /action] Error: Missing itemId');
    return res.status(400).json({ error: 'Missing required field: itemId' });
  }

  const id = parseInt(itemId);
  if (isNaN(id) || id <= 0) {
    console.log('[POST /action] Error: Invalid itemId');
    return res.status(400).json({ error: 'Invalid itemId' });
  }

  const item = items.find(i => i.id === id);
  if (!item) {
    console.log(`[POST /action] Error: Item not found (ID: ${id})`);
    return res.status(404).json({ error: 'Item not found' });
  }

  if (status) item.status = String(status);
  if (owner !== undefined) item.owner = String(owner);
  item.value2 = (item.value2 || 0) + 1;

  console.log(`[POST /action] Updated: ${item.name} (Status: ${item.status}, Owner: ${item.owner})`);

  res.json(item);
});

const PORT = 3001;
server.listen(PORT, '0.0.0.0', () => {
  console.log(`[Server] Running on port ${PORT}`);
  console.log(`[Server] Endpoints:`);
  console.log(`  POST   /item - Create item`);
  console.log(`  GET    /all - Get all items`);
  console.log(`  GET    /items/:owner - Get items by owner`);
  console.log(`  GET    /available - Get available items`);
  console.log(`  DELETE /item/:id - Delete item`);
  console.log(`  POST   /action - Update item status/owner`);
});
