import express, { type Express } from "express";
import cors from "cors";
import { InMemoryRepository } from "./repository/InMemoryRepository.js";
import { teamController } from "./controllers/teamController.js";
import { playerController } from "./controllers/playerController.js";
import { matchController } from "./controllers/matchController.js";
import { reportController } from "./controllers/reportController.js";
import { seed } from "./repository/seed.js";

export function createApp(repo: InMemoryRepository): Express {
  const app = express();
  app.use(cors());
  app.use(express.json());

  const teams = teamController(repo);
  const players = playerController(repo);
  const matches = matchController(repo);
  const reports = reportController(repo);

  app.get("/api/health", (_req, res) => res.json({ status: "ok" }));
  app.post("/api/reset", (_req, res) => {
    seed(repo);
    res.json({ ok: true });
  });

  app.get("/api/teams", teams.list);
  app.get("/api/teams/:id", teams.get);
  app.post("/api/teams", teams.create);
  app.put("/api/teams/:id", teams.update);
  app.delete("/api/teams/:id", teams.remove);

  app.get("/api/players", players.list);
  app.get("/api/players/:id", players.get);
  app.post("/api/players", players.create);
  app.put("/api/players/:id", players.update);
  app.delete("/api/players/:id", players.remove);

  app.get("/api/matches", matches.list);
  app.get("/api/matches/:id", matches.get);
  app.post("/api/matches", matches.create);
  app.put("/api/matches/:id", matches.update);
  app.delete("/api/matches/:id", matches.remove);
  app.post("/api/matches/:id/result", matches.result);

  app.get("/api/reports/top-scorers", reports.topScorers);
  app.get("/api/reports/standings", reports.standings);

  return app;
}
