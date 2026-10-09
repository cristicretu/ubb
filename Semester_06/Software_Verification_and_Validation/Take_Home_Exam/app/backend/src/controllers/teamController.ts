import type { Request, Response } from "express";
import type { InMemoryRepository } from "../repository/InMemoryRepository.js";
import { validateTeamInput } from "../models/Team.js";

export const teamController = (repo: InMemoryRepository) => ({
  list: (_req: Request, res: Response) => {
    res.json(repo.listTeams());
  },
  get: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    const team = repo.getTeam(id);
    if (!team) return res.status(404).json({ error: "team not found" });
    res.json(team);
  },
  create: (req: Request, res: Response) => {
    const errors = validateTeamInput(req.body);
    if (errors.length) return res.status(400).json({ errors });
    const team = repo.createTeam(req.body);
    res.status(201).json(team);
  },
  update: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    const errors = validateTeamInput(req.body);
    if (errors.length) return res.status(400).json({ errors });
    const team = repo.updateTeam(id, req.body);
    if (!team) return res.status(404).json({ error: "team not found" });
    res.json(team);
  },
  remove: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    if (!repo.getTeam(id))
      return res.status(404).json({ error: "team not found" });
    const ok = repo.deleteTeam(id);
    if (!ok)
      return res
        .status(409)
        .json({ error: "team has players or matches; remove them first" });
    res.status(204).send();
  },
});
