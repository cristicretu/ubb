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

const games = [];
let nextId = 1;

const seedData = [
  { name: 'Chess', status: 'available', user: '', size: 150, popularityScore: 12 },
  { name: 'Monopoly', status: 'borrowed', user: 'Alex', size: 850, popularityScore: 28 },
  { name: 'Scrabble', status: 'available', user: '', size: 320, popularityScore: 5 },
  { name: 'Risk', status: 'borrowed', user: 'Bob', size: 1200, popularityScore: 18 },
  { name: 'Catan', status: 'available', user: '', size: 650, popularityScore: 35 },
  { name: 'Ticket to Ride', status: 'borrowed', user: 'Carol', size: 980, popularityScore: 42 },
  { name: 'Pandemic', status: 'missing', user: '', size: 750, popularityScore: 15 },
  { name: 'Settlers of Catan', status: 'available', user: '', size: 680, popularityScore: 8 },
  { name: 'Codenames', status: 'borrowed', user: 'Diana', size: 420, popularityScore: 22 },
  { name: 'Azul', status: 'available', user: '', size: 550, popularityScore: 3 },
  { name: 'Wingspan', status: 'borrowed', user: 'Alex', size: 1100, popularityScore: 50 },
  { name: 'Gloomhaven', status: 'missing', user: '', size: 2800, popularityScore: 7 },
  { name: 'Terraforming Mars', status: 'available', user: '', size: 1350, popularityScore: 19 },
  { name: 'Splendor', status: 'borrowed', user: 'Bob', size: 380, popularityScore: 31 },
  { name: '7 Wonders', status: 'available', user: '', size: 720, popularityScore: 14 },
  { name: 'Dominion', status: 'borrowed', user: 'Carol', size: 590, popularityScore: 26 },
  { name: 'Carcassonne', status: 'available', user: '', size: 480, popularityScore: 11 },
  { name: 'Agricola', status: 'missing', user: '', size: 1650, popularityScore: 9 }
];

seedData.forEach(game => games.push({ id: nextId++, ...game }));
console.log(`[Server] Initialized with ${games.length} games`);

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

app.post('/game', (req, res) => {
  const { name, status, user, size } = req.body;

  if (!name || !status || size === undefined) {
    console.log('[POST /game] Error: Missing required fields');
    return res.status(400).json({ error: 'Missing required fields: name, status, size' });
  }

  const sizeInt = parseInt(size);
  if (isNaN(sizeInt) || sizeInt <= 0) {
    console.log('[POST /game] Error: Invalid size');
    return res.status(400).json({ error: 'Invalid size: must be a positive integer' });
  }

  const validStatuses = ['available', 'missing', 'canceled', 'borrowed'];
  if (!validStatuses.includes(status)) {
    console.log('[POST /game] Error: Invalid status');
    return res.status(400).json({ error: `Invalid status: must be one of ${validStatuses.join(', ')}` });
  }

  const newGame = {
    id: nextId++,
    name: String(name),
    status: String(status),
    user: user ? String(user) : '',
    size: sizeInt,
    popularityScore: 0
  };

  games.push(newGame);
  console.log(`[POST /game] Created game: ${newGame.name} (ID: ${newGame.id}, Status: ${newGame.status})`);

  broadcast(newGame);
  res.json(newGame);
});

app.get('/allGames', (req, res) => {
  console.log(`[GET /allGames] Returning ${games.length} games`);
  res.json(games);
});

app.get('/games/:user', (req, res) => {
  const userParam = req.params.user;
  const filtered = games.filter(game => 
    game.user.toLowerCase() === userParam.toLowerCase()
  );
  console.log(`[GET /games/${userParam}] Found ${filtered.length} games`);
  res.json(filtered);
});

app.get('/ready', (req, res) => {
  const available = games.filter(game => 
    game.status === 'available' && game.user === ''
  );
  console.log(`[GET /ready] Found ${available.length} available games`);
  res.json(available);
});

app.post('/book', (req, res) => {
  const { gameId, user } = req.body;

  if (!gameId || !user) {
    console.log('[POST /book] Error: Missing required fields');
    return res.status(400).json({ error: 'Missing required fields: gameId, user' });
  }

  const id = parseInt(gameId);
  if (isNaN(id) || id <= 0) {
    console.log('[POST /book] Error: Invalid gameId');
    return res.status(400).json({ error: 'Invalid gameId: must be a positive integer' });
  }

  const game = games.find(g => g.id === id);
  if (!game) {
    console.log(`[POST /book] Error: Game not found (ID: ${id})`);
    return res.status(404).json({ error: 'Game not found' });
  }

  game.status = 'borrowed';
  game.user = String(user);
  game.popularityScore = (game.popularityScore || 0) + 1;

  console.log(`[POST /book] Booked game: ${game.name} (ID: ${game.id}) by ${game.user} (Popularity: ${game.popularityScore})`);

  broadcast(game);
  res.json(game);
});

const PORT = 3001;
server.listen(PORT, '0.0.0.0', () => {
  console.log(`[Server] Running on port ${PORT}`);
  console.log(`[Server] Available endpoints:`);
  console.log(`  POST   /game - Create a new game`);
  console.log(`  GET    /allGames - Get all games`);
  console.log(`  GET    /games/:user - Get games borrowed by user`);
  console.log(`  GET    /ready - Get available games`);
  console.log(`  POST   /book - Borrow a game`);
});
