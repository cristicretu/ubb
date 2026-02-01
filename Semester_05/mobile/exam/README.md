# Mobile Exam Template Guide

## Quick Start

```bash
cd master/server
pnpm install
pnpm start

cd master/frontend
pnpm install
pnpm start
```

---

## Exam Structure (Always the Same)

Every exam follows this **exact pattern**:

### Entity (6-7 fields)
| Field | Type | Examples |
|-------|------|----------|
| id | Integer | Auto-generated |
| name | String | name, model, details |
| status | String | available, pending, done, open, borrowed |
| owner | String | user, student, client, owner, driver |
| value1 | Integer | size, cost, time, eCost |
| value2 | Integer | usage, popularity, capacity |

### Points Distribution (Always 10p total)
| Feature | Points |
|---------|--------|
| Save name in settings | 1p |
| Create item (online + offline) | 1p |
| View my items (cache + retry) | 2p |
| View filtered list (online only) | 1p |
| Action on item (delete/change/book) | 1p |
| Report (top N sorted) | 1p |
| WebSocket notification | 1p |
| Progress indicators | 0.5p |
| Error handling + logging | 0.5p |

---

## Template Structure

```
master/
├── server/
│   ├── index.js              # All endpoints + WebSocket
│   └── package.json
└── frontend/
    ├── config.ts             # ⚡ CHANGE: Server IP here
    ├── types/item.ts         # ⚡ CHANGE: Entity fields
    ├── utils/
    │   ├── api.ts            # ⚡ CHANGE: Endpoint names
    │   └── storage.ts        # ⚡ CHANGE: Storage keys
    ├── app/(tabs)/
    │   ├── _layout.tsx       # ⚡ CHANGE: Tab names/icons
    │   ├── index.tsx         # Tab 1: My Section
    │   ├── manage.tsx        # Tab 2: Manage Section
    │   └── reports.tsx       # Tab 3: Reports Section
    └── ...
```

---

## How to Adapt for Any Exam

### Step 1: Update Entity (`types/item.ts`)

Change field names to match exam:

```typescript
// FROM (template)
export interface Item {
  id: number;
  name: string;
  status: string;
  owner: string;
  value1: number;
  value2: number;
}

// TO (example: Games App)
export interface Game {
  id: number;
  name: string;
  status: string;      // available, borrowed, missing
  user: string;        // who borrowed
  size: number;        // in KB
  popularityScore: number;
}
```

### Step 2: Update API (`utils/api.ts`)

Change endpoint names and import:

```typescript
// Change import
import { Game } from '../types/game';

// Change function names and endpoints
export const createGame = (game: Omit<Game, 'id' | 'popularityScore'>) =>
  request<Game>('/game', { method: 'POST', body: JSON.stringify(game) });

export const getAllGames = () => request<Game[]>('/allGames');

export const getGamesByUser = (user: string) => 
  request<Game[]>(`/games/${encodeURIComponent(user)}`);

export const getAvailableGames = () => request<Game[]>('/ready');

export const bookGame = (gameId: number, user: string) =>
  request<Game>('/book', { method: 'POST', body: JSON.stringify({ gameId, user }) });
```

### Step 3: Update Storage (`utils/storage.ts`)

Change keys and import:

```typescript
import { Game } from '../types/game';

const KEYS = {
  ITEMS: '@games_app:games',
  PENDING: '@games_app:pending',
  OWNER: '@games_app:user',
};

// Rename functions: saveItems → saveGames, etc.
```

### Step 4: Update Server (`server/index.js`)

Change:
- Entity fields in seed data
- Endpoint names (`/item` → `/game`)
- Validation logic
- Status values

### Step 5: Update Screens

In each tab file:
- Change imports
- Change field names in forms
- Change field names in lists
- Update WebSocket alert message

### Step 6: Update Tab Layout (`_layout.tsx`)

Change tab names and icons:

```typescript
<Tabs.Screen
  name="index"
  options={{
    title: 'User',           // Change title
    headerTitle: 'User Section',
    tabBarIcon: ({ color, size }) => 
      <Ionicons name="person" size={size} color={color} />,
  }}
/>
```

---

## Common Endpoint Patterns

| Exam Type | Create | Get All | Get Filtered | Action |
|-----------|--------|---------|--------------|--------|
| Files | POST /file | GET /all | GET /files/:location | DELETE /file/:id |
| Games | POST /game | GET /allGames | GET /ready | POST /book |
| Documents | POST /document | GET /all | GET /documents/:owner | DELETE /document/:id |
| Orders | POST /order | GET /all | GET /pending | POST /status |
| Cabs | POST /cab | GET /all | GET /cabs/:color | DELETE /cab/:id |

---

## Common Report Types

### Top N by Field (Descending)
```typescript
const top10 = [...items]
  .sort((a, b) => b.value2 - a.value2)
  .slice(0, 10);
```

### Top N by Field (Ascending)
```typescript
const top10 = [...items]
  .sort((a, b) => a.value1 - b.value1)
  .slice(0, 10);
```

### Top N Owners by Count
```typescript
const ownerCounts = new Map<string, number>();
items.forEach(item => {
  const count = ownerCounts.get(item.owner) || 0;
  ownerCounts.set(item.owner, count + 1);
});

const topOwners = Array.from(ownerCounts.entries())
  .map(([owner, count]) => ({ owner, count }))
  .sort((a, b) => b.count - a.count)
  .slice(0, 10);
```

---

## WebSocket Alert Format

Always show 3 fields in human-readable form:

```typescript
useWebSocket({
  onMessage: (message: any) => {
    if (message.name && message.value1 && message.owner) {
      Alert.alert(
        'New Item',
        `Name: ${message.name}\nValue: ${message.value1}\nOwner: ${message.owner}`
      );
      loadItems(); // Refresh
    }
  },
});
```

---

## Postman Testing

### Create Item
```
POST http://localhost:3001/item
Content-Type: application/json

{
  "name": "Test Item",
  "status": "available",
  "owner": "Alice",
  "value1": 500
}
```

### Test WebSocket
1. Open Postman → New → WebSocket Request
2. URL: `ws://localhost:3001`
3. Click Connect
4. Send POST request from another tab
5. See message appear in WebSocket tab

---

## Checklist Before Submitting

- [ ] Server starts without errors
- [ ] Frontend connects to server (check config.ts IP)
- [ ] Can create item online
- [ ] Can create item offline (saves to pending)
- [ ] My items list shows and caches
- [ ] Offline shows cached data + retry button
- [ ] Available/filtered list works (online only)
- [ ] Delete/action works (online only)
- [ ] Reports show sorted data
- [ ] WebSocket alert appears on ALL tabs
- [ ] Loading spinners show during operations
- [ ] Errors show in Alert

---

## Troubleshooting

### "Network request failed"
- Check `config.ts` has correct IP
- For iOS simulator: use `localhost`
- For physical device: use your computer's IP

### "JSON Parse error: Unexpected character: <"
- Server not running or wrong port
- Run `pnpm start --clear` to clear cache

### WebSocket not working
- Make sure server uses same port for HTTP and WS
- Check useWebSocket hook uses ref pattern (not in dependency array)

### Items appearing twice
- Check for `justSubmittedId` ref pattern in index.tsx
- WebSocket + API response can both add same item

---

## Existing Templates

| Folder | Entity | Tabs |
|--------|--------|------|
| master | Item | 3 (My, Manage, Reports) |
| files_catalog | File | 2 (Record, Manage) |
| documents_app | Document | 3 (Owner, Manage, Status) |
| taxi_app | Cab | 4 (Registration, Manage, Reports, Driver) |
| games_app | Game | 3 (User, Selection, Status) |
