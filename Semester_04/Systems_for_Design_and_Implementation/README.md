# Systems for Design and Implementation (MPP)

Semester 4 · Year 2 · TypeScript, Next.js (T3 stack), tRPC, Prisma, PostgreSQL

Known locally as MPP (Medii de Proiectare și Programare). You build one full-stack web app over the semester and add a feature set each lab (CRUD, validation, charts, real-time updates, offline support, file upload, auth, logging, testing at scale), then build a smaller app in the practical exam.

## Contents

| Folder | What it is |
| --- | --- |
| [Project](Project) | Semester project: workout tracker where users record exercise videos and rate their form (bad/medium/good) |
| [Exam](Exam) | Practical exam: election candidates app with CRUD, a random candidate generator, live updates over SSE and a per-party chart |

Both folders were created with `create-t3-app` and still have its default README.

### Project features

- Exercise CRUD through Next.js API routes and tRPC, with filtering, sorting and infinite scroll
- Camera recording, video upload (including chunked uploads) and a gallery
- Charts and statistics (duration, quality, weekly progress)
- Background worker that generates fake exercises and pushes them to clients over WebSockets
- Offline mode: changes are stored locally and synced when the network returns
- Sign up / sign in with NextAuth, activity log, and an admin page that flags users with too many actions in a short window
- Jest tests for components and API routes; JMeter plan and shell scripts for stress testing

## How to run

Both apps need Node, pnpm and PostgreSQL.

```sh
cd Project            # or Exam
cp .env.example .env  # set DATABASE_URL and AUTH_SECRET
./start-database.sh   # starts Postgres in Docker, optional
pnpm install
pnpm db:push
pnpm db:seed          # Project only
pnpm dev
```

Tests (Project only):

```sh
pnpm test
```

## Notes

- The Exam app keeps candidates in memory (`src/app/api/candidates/route.ts`); they reset when the server restarts. The database is only used by the T3 auth scaffolding.
- `stress_test_monitoring.sh` uses a hardcoded upload URL (`http://localhost:3000/uploads/1746429384551_cfglsgm6.mp4`). Upload a video and replace it.
- The `*.js` / `*.sh` files in the Project root are helper scripts for testing the suspicious-activity monitor.
