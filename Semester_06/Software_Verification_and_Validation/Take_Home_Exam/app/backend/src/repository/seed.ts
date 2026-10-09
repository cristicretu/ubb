import type { InMemoryRepository } from "./InMemoryRepository.js";

export function seed(repo: InMemoryRepository): void {
  repo.reset();

  // Teams (Group A in FIFA World Cup 2026 host group is illustrative; data is fictional/example)
  const usa = repo.createTeam({
    name: "United States",
    country: "USA",
    group: "A",
    coach: "Mauricio Pochettino",
    fifaRanking: 16,
  });
  const mex = repo.createTeam({
    name: "Mexico",
    country: "Mexico",
    group: "A",
    coach: "Javier Aguirre",
    fifaRanking: 19,
  });
  const can = repo.createTeam({
    name: "Canada",
    country: "Canada",
    group: "B",
    coach: "Jesse Marsch",
    fifaRanking: 48,
  });
  const arg = repo.createTeam({
    name: "Argentina",
    country: "Argentina",
    group: "C",
    coach: "Lionel Scaloni",
    fifaRanking: 1,
  });
  const fra = repo.createTeam({
    name: "France",
    country: "France",
    group: "D",
    coach: "Didier Deschamps",
    fifaRanking: 2,
  });
  const bra = repo.createTeam({
    name: "Brazil",
    country: "Brazil",
    group: "B",
    coach: "Dorival Junior",
    fifaRanking: 5,
  });

  // A small squad for some teams
  const messi = repo.createPlayer({
    teamId: arg.id, name: "Lionel Messi", position: "FWD",
    jerseyNumber: 10, dateOfBirth: "1987-06-24", marketValueMillions: 20,
  });
  const dybala = repo.createPlayer({
    teamId: arg.id, name: "Paulo Dybala", position: "FWD",
    jerseyNumber: 21, dateOfBirth: "1993-11-15", marketValueMillions: 35,
  });
  const emi = repo.createPlayer({
    teamId: arg.id, name: "Emiliano Martinez", position: "GK",
    jerseyNumber: 23, dateOfBirth: "1992-09-02", marketValueMillions: 30,
  });

  const mbappe = repo.createPlayer({
    teamId: fra.id, name: "Kylian Mbappe", position: "FWD",
    jerseyNumber: 10, dateOfBirth: "1998-12-20", marketValueMillions: 180,
  });
  const griez = repo.createPlayer({
    teamId: fra.id, name: "Antoine Griezmann", position: "MID",
    jerseyNumber: 7, dateOfBirth: "1991-03-21", marketValueMillions: 25,
  });

  const vini = repo.createPlayer({
    teamId: bra.id, name: "Vinicius Junior", position: "FWD",
    jerseyNumber: 11, dateOfBirth: "2000-07-12", marketValueMillions: 180,
  });
  const casemiro = repo.createPlayer({
    teamId: bra.id, name: "Casemiro", position: "MID",
    jerseyNumber: 5, dateOfBirth: "1992-02-23", marketValueMillions: 18,
  });

  const pulisic = repo.createPlayer({
    teamId: usa.id, name: "Christian Pulisic", position: "FWD",
    jerseyNumber: 10, dateOfBirth: "1998-09-18", marketValueMillions: 28,
  });

  const lozano = repo.createPlayer({
    teamId: mex.id, name: "Hirving Lozano", position: "FWD",
    jerseyNumber: 22, dateOfBirth: "1995-07-30", marketValueMillions: 18,
  });

  // Matches (some completed with goals, some scheduled)
  const m1 = repo.createMatch({
    homeTeamId: usa.id, awayTeamId: mex.id, stage: "Group",
    date: "2026-06-12", venue: "MetLife Stadium",
  });
  repo.recordMatchResult(m1.id, 2, 1, [
    { playerId: pulisic.id, teamId: usa.id, minute: 22 },
    { playerId: pulisic.id, teamId: usa.id, minute: 67 },
    { playerId: lozano.id, teamId: mex.id, minute: 81 },
  ]);

  const m2 = repo.createMatch({
    homeTeamId: arg.id, awayTeamId: bra.id, stage: "Group",
    date: "2026-06-15", venue: "AT&T Stadium",
  });
  repo.recordMatchResult(m2.id, 2, 2, [
    { playerId: messi.id, teamId: arg.id, minute: 10 },
    { playerId: vini.id, teamId: bra.id, minute: 35 },
    { playerId: dybala.id, teamId: arg.id, minute: 60 },
    { playerId: vini.id, teamId: bra.id, minute: 88 },
  ]);

  const m3 = repo.createMatch({
    homeTeamId: fra.id, awayTeamId: can.id, stage: "Group",
    date: "2026-06-20", venue: "BC Place",
  });
  repo.recordMatchResult(m3.id, 3, 0, [
    { playerId: mbappe.id, teamId: fra.id, minute: 18 },
    { playerId: mbappe.id, teamId: fra.id, minute: 54 },
    { playerId: griez.id, teamId: fra.id, minute: 77 },
  ]);

  repo.createMatch({
    homeTeamId: arg.id, awayTeamId: fra.id, stage: "Final",
    date: "2026-07-19", venue: "MetLife Stadium",
  });
}
