import type { Request, Response } from "express";
import type { InMemoryRepository } from "../repository/InMemoryRepository.js";
import { topScorersReport, groupStandings } from "../services/reportService.js";

export const reportController = (repo: InMemoryRepository) => ({
  topScorers: (req: Request, res: Response) => {
    const limit = req.query.limit ? Number(req.query.limit) : undefined;
    const minGoals = req.query.minGoals ? Number(req.query.minGoals) : undefined;
    const report = topScorersReport(
      repo.listMatches(),
      repo.listPlayers(),
      repo.listTeams(),
      { limit, minGoals },
    );
    res.json(report);
  },
  standings: (_req: Request, res: Response) => {
    res.json(groupStandings(repo.listMatches(), repo.listTeams()));
  },
});
