import type { Request, Response } from "express";
import type { InMemoryRepository } from "../repository/InMemoryRepository.js";
import { validatePlayerInput } from "../models/Player.js";

export const playerController = (repo: InMemoryRepository) => ({
  list: (req: Request, res: Response) => {
    const teamId = req.query.teamId ? Number(req.query.teamId) : undefined;
    if (teamId !== undefined) {
      return res.json(repo.listPlayersByTeam(teamId));
    }
    res.json(repo.listPlayers());
  },
  get: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    const player = repo.getPlayer(id);
    if (!player) return res.status(404).json({ error: "player not found" });
    res.json(player);
  },
  create: (req: Request, res: Response) => {
    const errors = validatePlayerInput(req.body);
    if (errors.length) return res.status(400).json({ errors });
    if (!repo.getTeam(req.body.teamId))
      return res.status(400).json({ errors: ["teamId references a non-existent team"] });
    const player = repo.createPlayer(req.body);
    res.status(201).json(player);
  },
  update: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    const errors = validatePlayerInput(req.body);
    if (errors.length) return res.status(400).json({ errors });
    if (!repo.getTeam(req.body.teamId))
      return res.status(400).json({ errors: ["teamId references a non-existent team"] });
    const player = repo.updatePlayer(id, req.body);
    if (!player) return res.status(404).json({ error: "player not found" });
    res.json(player);
  },
  remove: (req: Request, res: Response) => {
    const id = Number(req.params.id);
    if (!repo.getPlayer(id))
      return res.status(404).json({ error: "player not found" });
    const ok = repo.deletePlayer(id);
    if (!ok)
      return res
        .status(409)
        .json({ error: "player has goals recorded; cannot delete" });
    res.status(204).send();
  },
});
