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

const orders = [];
let nextId = 1;

const seedData = [
  { table: 'Table 1', details: '2x Pizza', status: 'recorded', time: 600, type: 'normal' },
  { table: 'Table 2', details: '1x Steak', status: 'preparing', time: 900, type: 'normal' },
  { table: 'Table 3', details: '3x Beer', status: 'ready', time: 300, type: 'normal' },
  { table: 'VIP 1', details: '1x Pasta, 2x Wine', status: 'recorded', time: 1200, type: 'normal' },
  { table: 'Terrace A', details: '4x Burger', status: 'preparing', time: 750, type: 'normal' },
  { table: 'Table 5', details: '1x Salad', status: 'ready', time: 450, type: 'normal' },
  { table: 'Table 2', details: '2x Coffee', status: 'recorded', time: 300, type: 'normal' },
  { table: 'VIP 2', details: '1x Lobster', status: 'preparing', time: 1800, type: 'normal' },
  { table: 'Table 1', details: '1x Dessert', status: 'ready', time: 400, type: 'normal' },
  { table: 'Terrace B', details: '3x Pizza', status: 'recorded', time: 900, type: 'normal' },
  { table: 'Table 4', details: '2x Sushi', status: 'preparing', time: 1050, type: 'normal' },
  { table: 'Table 6', details: '1x Soup', status: 'ready', time: 500, type: 'normal' },
  { table: 'Delivery 1', details: '2x Pizza, 1x Coke', status: 'recorded', time: 1200, type: 'delivery' },
  { table: 'Table 3', details: '1x Sandwich', status: 'preparing', time: 600, type: 'normal' },
  { table: 'Delivery 2', details: '1x Burger, 2x Fries', status: 'ready', time: 800, type: 'delivery' },
];

seedData.forEach(order => orders.push({ id: nextId++, ...order }));
console.log(`[Server] Initialized with ${orders.length} orders`);

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

app.post('/order', (req, res) => {
  const { table, details, status, time, type } = req.body;

  if (!table || !details || !status || time === undefined || !type) {
    console.log('[POST /order] Error: Missing required fields');
    return res.status(400).json({ error: 'Missing required fields: table, details, status, time, type' });
  }

  const timeInt = parseInt(time);
  if (isNaN(timeInt) || timeInt <= 0) {
    console.log('[POST /order] Error: Invalid time');
    return res.status(400).json({ error: 'Invalid time: must be a positive integer' });
  }

  const validStatuses = ['recorded', 'preparing', 'ready', 'canceled'];
  if (!validStatuses.includes(status)) {
    console.log('[POST /order] Error: Invalid status');
    return res.status(400).json({ error: `Invalid status: must be one of ${validStatuses.join(', ')}` });
  }

  const validTypes = ['normal', 'delivery'];
  if (!validTypes.includes(type)) {
    console.log('[POST /order] Error: Invalid type');
    return res.status(400).json({ error: `Invalid type: must be one of ${validTypes.join(', ')}` });
  }

  const newOrder = {
    id: nextId++,
    table: String(table),
    details: String(details),
    status: String(status),
    time: timeInt,
    type: String(type)
  };

  orders.push(newOrder);
  console.log(`[POST /order] Created order: ID ${newOrder.id}, Table: ${newOrder.table}, Status: ${newOrder.status}`);

  broadcast(newOrder);
  res.json(newOrder);
});

app.get('/orders', (req, res) => {
  const readyOrders = orders.filter(order => order.status === 'ready');
  console.log(`[GET /orders] Returning ${readyOrders.length} orders with status "ready"`);
  res.json(readyOrders);
});

app.get('/order/:id', (req, res) => {
  const id = parseInt(req.params.id);

  if (isNaN(id) || id <= 0) {
    console.log(`[GET /order/${req.params.id}] Error: Invalid ID`);
    return res.status(400).json({ error: 'Invalid ID: must be a positive integer' });
  }

  const order = orders.find(o => o.id === id);
  if (!order) {
    console.log(`[GET /order/${id}] Error: Order not found`);
    return res.status(404).json({ error: 'Order not found' });
  }

  console.log(`[GET /order/${id}] Returning order: Table ${order.table}, Status: ${order.status}`);
  res.json(order);
});

app.get('/recorded', (req, res) => {
  const recordedOrders = orders.filter(order => order.status === 'recorded');
  console.log(`[GET /recorded] Returning ${recordedOrders.length} orders with status "recorded"`);
  res.json(recordedOrders);
});

app.post('/status', (req, res) => {
  const { orderId, status } = req.body;

  if (!orderId || !status) {
    console.log('[POST /status] Error: Missing required fields');
    return res.status(400).json({ error: 'Missing required fields: orderId, status' });
  }

  const id = parseInt(orderId);
  if (isNaN(id) || id <= 0) {
    console.log(`[POST /status] Error: Invalid orderId`);
    return res.status(400).json({ error: 'Invalid orderId: must be a positive integer' });
  }

  const validStatuses = ['recorded', 'preparing', 'ready', 'canceled'];
  if (!validStatuses.includes(status)) {
    console.log('[POST /status] Error: Invalid status');
    return res.status(400).json({ error: `Invalid status: must be one of ${validStatuses.join(', ')}` });
  }

  const order = orders.find(o => o.id === id);
  if (!order) {
    console.log(`[POST /status] Error: Order not found`);
    return res.status(404).json({ error: 'Order not found' });
  }

  order.status = String(status);
  console.log(`[POST /status] Updated order ${id} status to: ${status}`);

  broadcast(order);
  res.json(order);
});

app.get('/my/:table', (req, res) => {
  const tableParam = req.params.table;
  const tableOrders = orders.filter(order => 
    order.table.toLowerCase() === tableParam.toLowerCase()
  );

  if (tableOrders.length === 0) {
    console.log(`[GET /my/${tableParam}] No orders found for table`);
    return res.status(404).json({ error: 'No orders found for this table' });
  }

  const latestOrder = tableOrders.reduce((latest, current) => 
    current.id > latest.id ? current : latest
  );

  console.log(`[GET /my/${tableParam}] Returning latest order: ID ${latestOrder.id}, Status: ${latestOrder.status}`);
  res.json(latestOrder);
});

const PORT = 3001;
server.listen(PORT, '0.0.0.0', () => {
  console.log(`[Server] Running on port ${PORT}`);
  console.log(`[Server] Available endpoints:`);
  console.log(`  POST   /order - Create a new order`);
  console.log(`  GET    /orders - Get orders with status "ready"`);
  console.log(`  GET    /order/:id - Get single order by ID`);
  console.log(`  GET    /recorded - Get orders with status "recorded"`);
  console.log(`  POST   /status - Update order status`);
  console.log(`  GET    /my/:table - Get latest order for table`);
});
