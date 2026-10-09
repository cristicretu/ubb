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

const cabs = [];
let nextId = 1;

const seedData = [
  { name: 'Toyota Camry', status: 'working', size: 4, driver: 'John Smith', color: 'Red', capacity: 450 },
  { name: 'Honda Accord', status: 'new', size: 5, driver: 'Maria Garcia', color: 'Blue', capacity: 380 },
  { name: 'Ford Fusion', status: 'working', size: 4, driver: 'David Lee', color: 'Black', capacity: 420 },
  { name: 'Chevrolet Malibu', status: 'damaged', size: 5, driver: 'Sarah Johnson', color: 'White', capacity: 400 },
  { name: 'Nissan Altima', status: 'working', size: 4, driver: 'Michael Brown', color: 'Silver', capacity: 350 },
  { name: 'Hyundai Sonata', status: 'private', size: 5, driver: 'Emily Davis', color: 'Red', capacity: 480 },
  { name: 'Kia Optima', status: 'new', size: 4, driver: 'James Wilson', color: 'Blue', capacity: 320 },
  { name: 'Mazda 6', status: 'working', size: 5, driver: 'Lisa Anderson', color: 'Black', capacity: 410 },
  { name: 'Subaru Legacy', status: 'working', size: 4, driver: 'Robert Taylor', color: 'White', capacity: 390 },
  { name: 'Volkswagen Passat', status: 'new', size: 5, driver: 'Jennifer Martinez', color: 'Silver', capacity: 440 },
  { name: 'Toyota Corolla', status: 'working', size: 4, driver: 'William Thomas', color: 'Red', capacity: 300 },
  { name: 'Honda Civic', status: 'working', size: 5, driver: 'Patricia Jackson', color: 'Blue', capacity: 280 },
  { name: 'Ford Focus', status: 'damaged', size: 4, driver: 'Richard White', color: 'Black', capacity: 250 },
  { name: 'Chevrolet Cruze', status: 'private', size: 5, driver: 'Barbara Harris', color: 'White', capacity: 290 },
  { name: 'Nissan Sentra', status: 'working', size: 4, driver: 'Joseph Martin', color: 'Silver', capacity: 270 },
  { name: 'Hyundai Elantra', status: 'new', size: 5, driver: 'Susan Thompson', color: 'Red', capacity: 310 },
  { name: 'Kia Forte', status: 'working', size: 4, driver: 'Thomas Garcia', color: 'Blue', capacity: 260 },
  { name: 'Mazda 3', status: 'working', size: 5, driver: 'Jessica Rodriguez', color: 'Black', capacity: 330 },
  { name: 'Subaru Impreza', status: 'new', size: 4, driver: 'Charles Lewis', color: 'White', capacity: 340 },
  { name: 'Volkswagen Jetta', status: 'working', size: 5, driver: 'Sarah Walker', color: 'Silver', capacity: 360 },
  { name: 'Toyota Prius', status: 'working', size: 4, driver: 'Daniel Hall', color: 'Red', capacity: 290 },
  { name: 'Honda Insight', status: 'private', size: 5, driver: 'Nancy Allen', color: 'Blue', capacity: 300 },
  { name: 'Ford C-Max', status: 'damaged', size: 4, driver: 'Matthew Young', color: 'Black', capacity: 280 },
  { name: 'Chevrolet Volt', status: 'new', size: 5, driver: 'Karen King', color: 'White', capacity: 320 },
  { name: 'Nissan Leaf', status: 'working', size: 4, driver: 'Anthony Wright', color: 'Silver', capacity: 310 },
  { name: 'Mercedes Sprinter', status: 'working', size: 8, driver: 'Paul Miller', color: 'Black', capacity: 750 },
  { name: 'Ford Transit', status: 'working', size: 8, driver: 'Linda Moore', color: 'White', capacity: 800 },
  { name: 'Chevrolet Express', status: 'new', size: 7, driver: 'Mark Taylor', color: 'Silver', capacity: 720 },
  { name: 'Toyota Sienna', status: 'working', size: 7, driver: 'Carol Clark', color: 'Red', capacity: 680 },
  { name: 'Honda Odyssey', status: 'working', size: 7, driver: 'Steven Lewis', color: 'Blue', capacity: 650 },
  { name: 'Dodge Grand Caravan', status: 'damaged', size: 7, driver: 'Betty Walker', color: 'Black', capacity: 600 },
  { name: 'Chrysler Pacifica', status: 'private', size: 7, driver: 'Andrew Hall', color: 'White', capacity: 640 },
  { name: 'Nissan NV200', status: 'working', size: 5, driver: 'Sharon Allen', color: 'Silver', capacity: 550 },
  { name: 'Smart Fortwo', status: 'working', size: 2, driver: 'Kevin Young', color: 'Red', capacity: 200 },
  { name: 'Fiat 500', status: 'new', size: 2, driver: 'Michelle King', color: 'Blue', capacity: 220 },
  { name: 'Mini Cooper', status: 'working', size: 2, driver: 'Brian Wright', color: 'Black', capacity: 210 },
  { name: 'Scion iQ', status: 'damaged', size: 2, driver: 'Kimberly Lopez', color: 'White', capacity: 200 },
];

seedData.forEach(cab => cabs.push({ id: nextId++, ...cab }));
console.log(`[Server] Initialized with ${cabs.length} cabs`);

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

app.post('/cab', (req, res) => {
  const { name, status, size, driver, color, capacity } = req.body;

  if (!name || !status || size === undefined || !driver || !color || capacity === undefined) {
    return res.status(400).json({ error: 'Missing fields' });
  }

  const validStatuses = ['new', 'working', 'damaged', 'private'];
  if (!validStatuses.includes(status)) {
    return res.status(400).json({ error: 'Invalid status' });
  }

  const sizeInt = parseInt(size);
  const capacityInt = parseInt(capacity);
  if (isNaN(sizeInt) || sizeInt <= 0 || isNaN(capacityInt) || capacityInt <= 0) {
    return res.status(400).json({ error: 'Invalid size or capacity' });
  }

  const newCab = {
    id: nextId++,
    name: String(name),
    status: String(status),
    size: sizeInt,
    driver: String(driver),
    color: String(color),
    capacity: capacityInt
  };

  cabs.push(newCab);
  console.log(`[POST /cab] Created cab ${newCab.id}: ${newCab.name} (${newCab.driver})`);

  broadcast(newCab);
  res.json(newCab);
});

app.get('/all', (req, res) => {
  console.log(`[GET /all] Returning ${cabs.length} cabs`);
  res.json(cabs);
});

app.get('/colors', (req, res) => {
  const colors = [...new Set(cabs.map(c => c.color))].sort();
  console.log(`[GET /colors] Found ${colors.length} unique colors: ${colors.join(', ')}`);
  res.json(colors);
});

app.get('/cabs/:color', (req, res) => {
  const color = req.params.color;
  const filtered = cabs.filter(c => c.color.toLowerCase() === color.toLowerCase());
  console.log(`[GET /cabs/${color}] Found ${filtered.length} cabs`);
  res.json(filtered);
});

app.delete('/cab/:id', (req, res) => {
  const id = parseInt(req.params.id);
  if (isNaN(id) || id <= 0) {
    return res.status(400).json({ error: 'Invalid ID' });
  }

  const index = cabs.findIndex(c => c.id === id);
  if (index === -1) {
    return res.status(404).json({ error: 'Not found' });
  }

  const deleted = cabs.splice(index, 1)[0];
  console.log(`[DELETE /cab/${id}] Deleted cab ${deleted.name} (${deleted.driver})`);
  res.json({ success: true, deleted });
});

app.get('/my/:driver', (req, res) => {
  const driver = req.params.driver;
  const filtered = cabs.filter(c => c.driver.toLowerCase() === driver.toLowerCase());
  console.log(`[GET /my/${driver}] Found ${filtered.length} cabs`);
  res.json(filtered);
});

const PORT = 3001;
server.listen(PORT, '0.0.0.0', () => console.log(`[Server] Running on port ${PORT}`));
