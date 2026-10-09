# Mobile Application Programming

Semester 5 · Year 3 · TypeScript, React Native (Expo), Node.js, Swift

You build a mobile CRUD app that works offline and syncs with a server, then pass a practical exam where you write a small client-server app with caching, offline creates and WebSocket notifications in limited time.

## Contents

| Folder | What it is |
| --- | --- |
| [Project](Project) | Course project: "gicu mecanicu", a car maintenance tracker (ITP and rovinieta deadlines, mileage). Has a SwiftUI version (`mobile-cars`) and an Expo version (`my-expo-app`) with SQLite via Drizzle, a sync queue, WebSocket updates and an Express server |
| [Exam](Exam) | Exam practice templates. See [Exam/README.md](Exam/README.md) for the full guide |
| [Exam/master](Exam/master) | Generic template: `Item` entity, 3 tabs (My items, Manage, Reports) |
| [Exam/files_catalog](Exam/files_catalog) | Files catalog: create, list by location, delete |
| [Exam/documents_app](Exam/documents_app) | Documents: create, list by owner, delete |
| [Exam/games_app](Exam/games_app) | Games: create, list ready games, book a game |
| [Exam/taxi_app](Exam/taxi_app) | Cabs: create, filter by color, delete, driver view |
| [Exam/recipe_app](Exam/recipe_app) | Recipes: list by type, create, delete, low-rated report, increment rating |
| [Exam/restaurant_app](Exam/restaurant_app) | Restaurant orders, server only (no frontend) |

Every exam app is a `server/` (Express + `ws`, in-memory data with seed values) and a `frontend/` (Expo Router with tabs, AsyncStorage cache, offline pending queue, WebSocket alerts).

## How to run

```sh
cd Exam/master/server
pnpm install
pnpm start

# in another terminal
cd Exam/master/frontend
pnpm install
pnpm start
```

Set the server IP in `frontend/config.ts`. Use `localhost` for the iOS simulator, your computer's LAN IP for a physical phone.

## Notes

- The Swift version of the project needs Xcode (macOS).
