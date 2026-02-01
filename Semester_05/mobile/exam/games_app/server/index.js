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

const documents = [];
let nextId = 1;

const seedData = [
  { name: 'Annual Report 2024', status: 'shared', owner: 'John', size: 1250, usage: 45 },
  { name: 'Project Proposal', status: 'draft', owner: 'Maria', size: 350, usage: 8 },
  { name: 'Financial Analysis', status: 'secret', owner: 'David', size: 890, usage: 3 },
  { name: 'Meeting Notes', status: 'open', owner: 'Sarah', size: 120, usage: 67 },
  { name: 'User Manual', status: 'shared', owner: 'Michael', size: 2100, usage: 156 },
  { name: 'Budget Planning', status: 'draft', owner: 'John', size: 450, usage: 12 },
  { name: 'Client Contract', status: 'secret', owner: 'Maria', size: 680, usage: 5 },
  { name: 'Marketing Strategy', status: 'open', owner: 'David', size: 950, usage: 89 },
  { name: 'Technical Documentation', status: 'shared', owner: 'Sarah', size: 3200, usage: 234 },
  { name: 'Quarterly Review', status: 'open', owner: 'Michael', size: 750, usage: 45 },
  { name: 'Research Paper', status: 'draft', owner: 'John', size: 1800, usage: 23 },
  { name: 'Product Specs', status: 'shared', owner: 'Maria', size: 1100, usage: 78 },
  { name: 'Confidential Memo', status: 'secret', owner: 'David', size: 250, usage: 2 },
  { name: 'Training Guide', status: 'open', owner: 'Sarah', size: 1450, usage: 112 },
  { name: 'Code Review', status: 'draft', owner: 'Michael', size: 520, usage: 15 },
  { name: 'Sales Report', status: 'shared', owner: 'John', size: 980, usage: 67 },
  { name: 'Design Mockups', status: 'open', owner: 'Maria', size: 4200, usage: 189 },
  { name: 'Legal Agreement', status: 'secret', owner: 'David', size: 650, usage: 4 },
  { name: 'Team Handbook', status: 'shared', owner: 'Sarah', size: 2800, usage: 201 },
  { name: 'Performance Metrics', status: 'open', owner: 'Michael', size: 380, usage: 34 },
];

seedData.forEach(doc => documents.push({ id: nextId++, ...doc }));
console.log(`[Server] Initialized with ${documents.length} documents`);

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

app.post('/document', (req, res) => {
  const { name, status, owner, size } = req.body;

  if (!name || !status || !owner || size === undefined) {
    console.log('[POST /document] Error: Missing required fields');
    return res.status(400).json({ error: 'Missing required fields: name, status, owner, size' });
  }

  const sizeInt = parseInt(size);
  if (isNaN(sizeInt) || sizeInt <= 0) {
    console.log('[POST /document] Error: Invalid size');
    return res.status(400).json({ error: 'Invalid size: must be a positive integer' });
  }

  const validStatuses = ['shared', 'open', 'draft', 'secret'];
  if (!validStatuses.includes(status)) {
    console.log('[POST /document] Error: Invalid status');
    return res.status(400).json({ error: `Invalid status: must be one of ${validStatuses.join(', ')}` });
  }

  const newDocument = {
    id: nextId++,
    name: String(name),
    status: String(status),
    owner: String(owner),
    size: sizeInt,
    usage: 0
  };

  documents.push(newDocument);
  console.log(`[POST /document] Created document: ${newDocument.name} (ID: ${newDocument.id}, Owner: ${newDocument.owner})`);

  broadcast(newDocument);
  res.json(newDocument);
});

app.get('/all', (req, res) => {
  console.log(`[GET /all] Returning ${documents.length} documents`);
  res.json(documents);
});

app.get('/documents/:owner', (req, res) => {
  const ownerParam = req.params.owner;
  const filtered = documents.filter(doc => 
    doc.owner.toLowerCase() === ownerParam.toLowerCase()
  );
  console.log(`[GET /documents/${ownerParam}] Found ${filtered.length} documents`);
  res.json(filtered);
});

app.delete('/document/:id', (req, res) => {
  const id = parseInt(req.params.id);
  
  if (isNaN(id) || id <= 0) {
    console.log(`[DELETE /document/${req.params.id}] Error: Invalid ID`);
    return res.status(400).json({ error: 'Invalid ID: must be a positive integer' });
  }

  const index = documents.findIndex(doc => doc.id === id);
  if (index === -1) {
    console.log(`[DELETE /document/${id}] Error: Document not found`);
    return res.status(404).json({ error: 'Document not found' });
  }

  const deleted = documents.splice(index, 1)[0];
  console.log(`[DELETE /document/${id}] Deleted document: ${deleted.name} (Owner: ${deleted.owner})`);

  broadcast({ type: 'deleted', id: deleted.id });
  res.json({ success: true, deleted });
});

const PORT = 3001;
server.listen(PORT, '0.0.0.0', () => {
  console.log(`[Server] Running on port ${PORT}`);
  console.log(`[Server] Available endpoints:`);
  console.log(`  POST   /document - Create a new document`);
  console.log(`  GET    /all - Get all documents`);
  console.log(`  GET    /documents/:owner - Get documents by owner`);
  console.log(`  DELETE /document/:id - Delete a document by ID`);
});
