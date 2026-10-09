import type { Request, Response } from "express";
import type { InMemoryRepository } from "../repository/InMemoryRepository.js";
import { validateMatchInput } from "../models/Match.js";
import { recordMatchResult } from "../services/matchResultService.js";

export const matchController = (repo: InMemoryRepository) => ({
  list: (_req: Request, res: Response) => {
    res.json(repo.listMatches());
  },
  get: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    const match = repo.getMatch(id);
    if (!match) return res.status(404).json({ error: "match not found" });
    res.json(match);
  },
  create: (req: Request, res: Response) => {
    const errors = validateMatchInput(req.body);
    if (errors.length) return res.status(400).json({ errors });
    if (!repo.getTeam(req.body.homeTeamId))
      return res.status(400).json({ errors: ["homeTeamId not found"] });
    if (!repo.getTeam(req.body.awayTeamId))
      return res.status(400).json({ errors: ["awayTeamId not found"] });
    const match = repo.createMatch(req.body);
    res.status(201).json(match);
  },
  update: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    const errors = validateMatchInput(req.body);
    if (errors.length) return res.status(400).json({ errors });
    const match = repo.updateMatch(id, req.body);
    if (!match) return res.status(404).json({ error: "match not found" });
    res.json(match);
  },
  remove: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    if (!repo.getMatch(id))
      return res.status(404).json({ error: "match not found" });
    repo.deleteMatch(id);
    res.status(204).send();
  },
  result: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    const outcome = recordMatchResult(repo, id, req.body);
    if (!outcome.ok) return res.status(400).json({ errors: outcome.errors });
    res.json(repo.getMatch(id));
  },
});
