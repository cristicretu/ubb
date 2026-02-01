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

const recipes = [];
let nextId = 1;

const seedData = [
  { name: 'Pasta Carbonara', details: 'Classic Italian pasta with eggs, cheese, pancetta, and black pepper', time: 900, type: 'medium', rating: 45 },
  { name: 'Chicken Curry', details: 'Spicy Indian curry with tender chicken and aromatic spices', time: 1800, type: 'medium', rating: 42 },
  { name: 'Caesar Salad', details: 'Fresh romaine lettuce with Caesar dressing, croutons, and parmesan', time: 600, type: 'beginner', rating: 38 },
  { name: 'Beef Wellington', details: 'Tender beef fillet wrapped in puff pastry with mushroom duxelles', time: 7200, type: 'advanced', rating: 48 },
  { name: 'Chocolate Chip Cookies', details: 'Soft and chewy cookies with melted chocolate chips', time: 1200, type: 'beginner', rating: 35 },
  { name: 'Sushi Rolls', details: 'Fresh fish and vegetables wrapped in seasoned rice and nori', time: 3600, type: 'advanced', rating: 50 },
  { name: 'Grilled Cheese Sandwich', details: 'Crispy bread with melted cheese', time: 300, type: 'beginner', rating: 15 },
  { name: 'Pad Thai', details: 'Stir-fried rice noodles with tamarind, fish sauce, and peanuts', time: 1500, type: 'medium', rating: 40 },
  { name: 'Tiramisu', details: 'Italian dessert with coffee-soaked ladyfingers and mascarpone', time: 2400, type: 'medium', rating: 44 },
  { name: 'French Onion Soup', details: 'Caramelized onions in rich beef broth with melted cheese', time: 2700, type: 'medium', rating: 39 },
  { name: 'Ramen Noodles', details: 'Japanese noodle soup with rich broth, eggs, and vegetables', time: 2100, type: 'medium', rating: 41 },
  { name: 'Apple Pie', details: 'Classic American dessert with spiced apples in flaky crust', time: 3300, type: 'medium', rating: 37 },
  { name: 'Scrambled Eggs', details: 'Creamy eggs cooked slowly with butter', time: 300, type: 'beginner', rating: 12 },
  { name: 'Coq au Vin', details: 'French chicken braised in wine with mushrooms and onions', time: 5400, type: 'advanced', rating: 46 },
  { name: 'Margherita Pizza', details: 'Simple pizza with tomato, mozzarella, and fresh basil', time: 1800, type: 'beginner', rating: 33 },
  { name: 'Beef Stroganoff', details: 'Tender beef in creamy mushroom sauce served over noodles', time: 2400, type: 'medium', rating: 43 },
  { name: 'Chocolate Soufflé', details: 'Light and airy chocolate dessert that rises perfectly', time: 2700, type: 'advanced', rating: 47 },
  { name: 'Fish Tacos', details: 'Crispy fish with fresh slaw and creamy sauce in soft tortillas', time: 1500, type: 'beginner', rating: 28 },
  { name: 'Ratatouille', details: 'French vegetable stew with eggplant, zucchini, and tomatoes', time: 3600, type: 'medium', rating: 36 },
  { name: 'Baked Mac and Cheese', details: 'Creamy pasta with multiple cheeses and crispy breadcrumb topping', time: 2100, type: 'beginner', rating: 25 }
];

seedData.forEach(recipe => recipes.push({ id: nextId++, ...recipe }));
console.log(`[Server] Initialized with ${recipes.length} recipes`);

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

app.get('/types', (req, res) => {
  const types = [...new Set(recipes.map(r => r.type))];
  console.log(`[GET /types] Returning ${types.length} unique types`);
  res.json(types);
});

app.get('/recipes/:type', (req, res) => {
  const typeParam = req.params.type.toLowerCase();
  const filtered = recipes.filter(recipe => 
    recipe.type.toLowerCase() === typeParam
  );
  console.log(`[GET /recipes/${req.params.type}] Found ${filtered.length} recipes`);
  res.json(filtered);
});

app.post('/recipe', (req, res) => {
  const { name, details, time, type, rating } = req.body;

  if (!name || !details || time === undefined || !type || rating === undefined) {
    console.log('[POST /recipe] Error: Missing required fields');
    return res.status(400).json({ error: 'Missing required fields: name, details, time, type, rating' });
  }

  const timeInt = parseInt(time);
  const ratingInt = parseInt(rating);

  if (isNaN(timeInt) || timeInt < 0) {
    console.log('[POST /recipe] Error: Invalid time');
    return res.status(400).json({ error: 'Invalid time: must be a non-negative integer' });
  }

  if (isNaN(ratingInt) || ratingInt < 0) {
    console.log('[POST /recipe] Error: Invalid rating');
    return res.status(400).json({ error: 'Invalid rating: must be a non-negative integer' });
  }

  const newRecipe = {
    id: nextId++,
    name: String(name),
    details: String(details),
    time: timeInt,
    type: String(type),
    rating: ratingInt
  };

  recipes.push(newRecipe);
  console.log(`[POST /recipe] Created: ${newRecipe.name} (ID: ${newRecipe.id})`);

  broadcast(newRecipe);
  res.json(newRecipe);
});

app.delete('/recipe/:id', (req, res) => {
  const id = parseInt(req.params.id);

  if (isNaN(id) || id <= 0) {
    console.log(`[DELETE /recipe/:id] Error: Invalid ID`);
    return res.status(400).json({ error: 'Invalid ID' });
  }

  const index = recipes.findIndex(recipe => recipe.id === id);
  if (index === -1) {
    console.log(`[DELETE /recipe/${id}] Error: Recipe not found`);
    return res.status(404).json({ error: 'Recipe not found' });
  }

  const deleted = recipes.splice(index, 1)[0];
  console.log(`[DELETE /recipe/${id}] Deleted: ${deleted.name}`);

  res.json({ success: true, deleted });
});

app.get('/low', (req, res) => {
  console.log(`[GET /low] Returning all ${recipes.length} recipes`);
  res.json(recipes);
});

app.post('/increment', (req, res) => {
  const { recipeId } = req.body;

  if (!recipeId) {
    console.log('[POST /increment] Error: Missing recipeId');
    return res.status(400).json({ error: 'Missing required field: recipeId' });
  }

  const id = parseInt(recipeId);
  if (isNaN(id) || id <= 0) {
    console.log('[POST /increment] Error: Invalid recipeId');
    return res.status(400).json({ error: 'Invalid recipeId' });
  }

  const recipe = recipes.find(r => r.id === id);
  if (!recipe) {
    console.log(`[POST /increment] Error: Recipe not found (ID: ${id})`);
    return res.status(404).json({ error: 'Recipe not found' });
  }

  recipe.rating += 1;
  console.log(`[POST /increment] Updated: ${recipe.name} (Rating: ${recipe.rating})`);

  res.json(recipe);
});

const PORT = 3001;
server.listen(PORT, '0.0.0.0', () => {
  console.log(`[Server] Running on port ${PORT}`);
  console.log(`[Server] Endpoints:`);
  console.log(`  GET    /types - Get all unique recipe types`);
  console.log(`  GET    /recipes/:type - Get recipes by type`);
  console.log(`  POST   /recipe - Create recipe`);
  console.log(`  DELETE /recipe/:id - Delete recipe`);
  console.log(`  GET    /low - Get all recipes`);
  console.log(`  POST   /increment - Increment recipe rating`);
});
