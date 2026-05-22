import { useEffect, useMemo, useState } from "react";
import { api, type Match, type Player, type Team } from "../api";

interface GoalDraft {
  playerId: number;
  minute: number;
}

export default function RecordResultPage() {
  const [matches, setMatches] = useState<Match[]>([]);
  const [teams, setTeams] = useState<Team[]>([]);
  const [players, setPlayers] = useState<Player[]>([]);
  const [matchId, setMatchId] = useState<number | "">("");
  const [homeScore, setHomeScore] = useState(0);
  const [awayScore, setAwayScore] = useState(0);
  const [homeGoals, setHomeGoals] = useState<GoalDraft[]>([]);
  const [awayGoals, setAwayGoals] = useState<GoalDraft[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [okMessage, setOkMessage] = useState<string | null>(null);

  const refresh = async () => {
    const [ms, ts, ps] = await Promise.all([
      api.listMatches(),
      api.listTeams(),
      api.listPlayers(),
    ]);
    setMatches(ms);
    setTeams(ts);
    setPlayers(ps);
  };
  useEffect(() => { refresh().catch((e) => setError(e.message)); }, []);

  const scheduled = useMemo(
    () => matches.filter((m) => m.status === "Scheduled"),
    [matches],
  );

  const selected = matches.find((m) => m.id === matchId) ?? null;
  const homeTeam = teams.find((t) => t.id === selected?.homeTeamId) ?? null;
  const awayTeam = teams.find((t) => t.id === selected?.awayTeamId) ?? null;
  const homePlayers = players.filter((p) => p.teamId === selected?.homeTeamId);
  const awayPlayers = players.filter((p) => p.teamId === selected?.awayTeamId);

  const adjust = (
    arr: GoalDraft[],
    setArr: (v: GoalDraft[]) => void,
    score: number,
    defaultPlayerId: number,
  ) => {
    if (score < 0) return;
    const next = [...arr];
    while (next.length < score) next.push({ playerId: defaultPlayerId, minute: 45 });
    while (next.length > score) next.pop();
    setArr(next);
  };

  useEffect(() => {
    if (!homePlayers[0]) return;
    adjust(homeGoals, setHomeGoals, homeScore, homePlayers[0].id);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [homeScore, selected?.id]);
  useEffect(() => {
    if (!awayPlayers[0]) return;
    adjust(awayGoals, setAwayGoals, awayScore, awayPlayers[0].id);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [awayScore, selected?.id]);

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setOkMessage(null);
    if (!matchId) {
      setError("pick a scheduled match");
      return;
    }
    try {
      await api.recordResult(Number(matchId), {
        homeScore, awayScore, homeGoals, awayGoals,
      });
      setOkMessage(`Match #${matchId} recorded.`);
      setMatchId("");
      setHomeGoals([]);
      setAwayGoals([]);
      setHomeScore(0);
      setAwayScore(0);
      await refresh();
    } catch (e: any) {
      setError(e.message);
    }
  };

  return (
    <form className="space-y-4 max-w-3xl" onSubmit={submit} data-testid="record-form">
      <div className="card space-y-2">
        <h2 className="text-lg font-bold">Record match result</h2>
        <p className="text-sm text-stone-600">
          Cross-entity functionality — uses Teams, Players and Matches together.
          Pick a scheduled match, set the final score, then assign each goal
          to the scoring player.
        </p>
        {error && <div className="rounded bg-red-50 border border-red-200 px-2 py-1 text-sm text-red-700" data-testid="record-error">{error}</div>}
        {okMessage && <div className="rounded bg-emerald-50 border border-emerald-200 px-2 py-1 text-sm text-emerald-800" data-testid="record-ok">{okMessage}</div>}
        <div>
          <label className="label">Scheduled match</label>
          <select
            className="input"
            value={matchId}
            onChange={(e) => setMatchId(e.target.value ? Number(e.target.value) : "")}
            data-testid="record-match-select"
          >
            <option value="">— choose —</option>
            {scheduled.map((m) => {
              const home = teams.find((t) => t.id === m.homeTeamId)?.name ?? "?";
              const away = teams.find((t) => t.id === m.awayTeamId)?.name ?? "?";
              return (
                <option key={m.id} value={m.id}>
                  #{m.id} · {m.stage} · {m.date} · {home} vs {away} ({m.venue})
                </option>
              );
            })}
          </select>
        </div>
      </div>

      {selected && homeTeam && awayTeam && (
        <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
          <SideEditor
            label={homeTeam.name}
            score={homeScore}
            setScore={setHomeScore}
            goals={homeGoals}
            setGoals={setHomeGoals}
            players={homePlayers}
            testId="record-home"
          />
          <SideEditor
            label={awayTeam.name}
            score={awayScore}
            setScore={setAwayScore}
            goals={awayGoals}
            setGoals={setAwayGoals}
            players={awayPlayers}
            testId="record-away"
          />
        </div>
      )}

      {selected && (
        <button type="submit" className="btn-primary" data-action="submit-result">
          Save result
        </button>
      )}
    </form>
  );
}

function SideEditor({
  label, score, setScore, goals, setGoals, players, testId,
}: {
  label: string;
  score: number;
  setScore: (n: number) => void;
  goals: { playerId: number; minute: number }[];
  setGoals: (g: { playerId: number; minute: number }[]) => void;
  players: Player[];
  testId: string;
}) {
  return (
    <div className="card space-y-2" data-testid={testId}>
      <h3 className="font-bold">{label}</h3>
      <div>
        <label className="label">Goals scored</label>
        <input
          className="input w-24"
          type="number"
          min={0}
          max={20}
          value={score}
          onChange={(e) => setScore(Math.max(0, Number(e.target.value)))}
          data-testid={`${testId}-score`}
        />
      </div>
      {goals.length > 0 && (
        <table className="w-full text-sm">
          <thead>
            <tr className="text-left text-stone-500 border-b">
              <th className="py-1">#</th>
              <th>Scorer</th>
              <th>Minute</th>
            </tr>
          </thead>
          <tbody>
            {goals.map((g, i) => (
              <tr key={i} className="border-b last:border-b-0">
                <td className="py-1 text-stone-500">{i + 1}</td>
                <td>
                  <select
                    className="input"
                    value={g.playerId}
                    onChange={(e) => {
                      const next = [...goals];
                      next[i] = { ...next[i], playerId: Number(e.target.value) };
                      setGoals(next);
                    }}
                  >
                    {players.length === 0 && <option value={0}>No players</option>}
                    {players.map((p) => (
                      <option key={p.id} value={p.id}>{p.name} ({p.position})</option>
                    ))}
                  </select>
                </td>
                <td>
                  <input
                    className="input w-24"
                    type="number"
                    min={1}
                    max={120}
                    value={g.minute}
                    onChange={(e) => {
                      const next = [...goals];
                      next[i] = { ...next[i], minute: Number(e.target.value) };
                      setGoals(next);
                    }}
                  />
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </div>
  );
}
