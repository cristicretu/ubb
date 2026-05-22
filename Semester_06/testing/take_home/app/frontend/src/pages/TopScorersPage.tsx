import { useEffect, useState } from "react";
import { api, type TopScorerEntry } from "../api";

export default function TopScorersPage() {
  const [entries, setEntries] = useState<TopScorerEntry[]>([]);
  const [limit, setLimit] = useState(10);
  const [error, setError] = useState<string | null>(null);

  const refresh = () =>
    api.topScorers(limit).then(setEntries).catch((e) => setError(e.message));
  useEffect(() => { refresh(); }, [limit]);

  return (
    <div className="card max-w-3xl">
      <div className="flex items-center justify-between mb-3">
        <div>
          <h2 className="text-lg font-bold">Top scorers — World Cup 2026</h2>
          <p className="text-sm text-stone-600">
            Report joining Matches, Players and Teams. Updates when a result is
            recorded.
          </p>
        </div>
        <div className="flex items-center gap-2 text-sm">
          <label>Show top</label>
          <input
            className="input w-20"
            type="number"
            min={1}
            max={50}
            value={limit}
            onChange={(e) => setLimit(Math.max(1, Number(e.target.value)))}
            data-testid="top-scorers-limit"
          />
        </div>
      </div>
      {error && <div className="rounded bg-red-50 border border-red-200 px-2 py-1 text-sm text-red-700">{error}</div>}
      <table className="w-full text-sm" data-testid="top-scorers-table">
        <thead>
          <tr className="text-left text-stone-500 border-b">
            <th className="py-1">Rank</th>
            <th>Player</th>
            <th>Pos</th>
            <th>Team</th>
            <th>Country</th>
            <th className="text-right">Goals</th>
          </tr>
        </thead>
        <tbody>
          {entries.length === 0 && (
            <tr>
              <td colSpan={6} className="py-3 text-center text-stone-500 italic">
                no goals recorded yet
              </td>
            </tr>
          )}
          {entries.map((e, i) => (
            <tr key={e.playerId} className="border-b last:border-b-0">
              <td className="py-1.5 text-stone-500">{i + 1}</td>
              <td className="font-medium">{e.playerName}</td>
              <td>{e.position}</td>
              <td>{e.teamName}</td>
              <td>{e.country}</td>
              <td className="text-right font-mono">{e.goals}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
