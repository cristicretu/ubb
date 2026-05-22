import { useEffect, useState } from "react";
import { api, type Match, type Stage, type Team } from "../api";

const STAGES: Stage[] = ["Group", "R16", "QF", "SF", "Final"];

interface FormState {
  homeTeamId: number;
  awayTeamId: number;
  stage: Stage;
  date: string;
  venue: string;
}

const empty: FormState = {
  homeTeamId: 0,
  awayTeamId: 0,
  stage: "Group",
  date: "2026-06-12",
  venue: "MetLife Stadium",
};

export default function MatchesPage() {
  const [matches, setMatches] = useState<Match[]>([]);
  const [teams, setTeams] = useState<Team[]>([]);
  const [editing, setEditing] = useState<Match | null>(null);
  const [form, setForm] = useState<FormState>(empty);
  const [error, setError] = useState<string | null>(null);

  const refresh = async () => {
    const [ms, ts] = await Promise.all([api.listMatches(), api.listTeams()]);
    setMatches(ms);
    setTeams(ts);
    setForm((f) =>
      f.homeTeamId === 0 && ts.length >= 2
        ? { ...f, homeTeamId: ts[0].id, awayTeamId: ts[1].id }
        : f,
    );
  };
  useEffect(() => { refresh().catch((e) => setError(e.message)); }, []);

  const teamName = (id: number) => teams.find((t) => t.id === id)?.name ?? "?";

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    try {
      if (editing) await api.updateMatch(editing.id, form);
      else await api.createMatch(form);
      setEditing(null);
      setForm({ ...empty, homeTeamId: teams[0]?.id ?? 0, awayTeamId: teams[1]?.id ?? 0 });
      refresh();
    } catch (e: any) {
      setError(e.message);
    }
  };

  const remove = async (id: number) => {
    setError(null);
    try {
      await api.deleteMatch(id);
      refresh();
    } catch (e: any) {
      setError(e.message);
    }
  };

  const edit = (m: Match) => {
    setEditing(m);
    setForm({
      homeTeamId: m.homeTeamId,
      awayTeamId: m.awayTeamId,
      stage: m.stage,
      date: m.date,
      venue: m.venue,
    });
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
      <div className="md:col-span-2 card">
        <h2 className="text-lg font-bold mb-3">Matches</h2>
        <table className="w-full text-sm" data-testid="matches-table">
          <thead>
            <tr className="text-left text-stone-500 border-b">
              <th className="py-1">#</th>
              <th>Date</th>
              <th>Stage</th>
              <th>Home</th>
              <th>Score</th>
              <th>Away</th>
              <th>Venue</th>
              <th></th>
            </tr>
          </thead>
          <tbody>
            {matches.map((m) => (
              <tr key={m.id} className="border-b last:border-b-0">
                <td className="py-1.5 text-stone-500">{m.id}</td>
                <td>{m.date}</td>
                <td>{m.stage}</td>
                <td className="font-medium text-right">{teamName(m.homeTeamId)}</td>
                <td className="text-center font-mono">
                  {m.status === "Completed"
                    ? `${m.homeScore} – ${m.awayScore}`
                    : <span className="text-stone-400 text-xs italic">scheduled</span>}
                </td>
                <td className="font-medium">{teamName(m.awayTeamId)}</td>
                <td className="text-stone-600">{m.venue}</td>
                <td className="text-right space-x-1">
                  <button className="btn" onClick={() => edit(m)} data-action="edit-match">Edit</button>
                  <button className="btn-danger" onClick={() => remove(m.id)} data-action="delete-match">Delete</button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      <form className="card space-y-3" onSubmit={submit} data-testid="match-form">
        <h2 className="text-lg font-bold">{editing ? `Edit match #${editing.id}` : "Schedule new match"}</h2>
        {error && <div className="rounded bg-red-50 border border-red-200 px-2 py-1 text-sm text-red-700" data-testid="match-error">{error}</div>}
        <div className="grid grid-cols-2 gap-3">
          <div>
            <label className="label">Home team</label>
            <select className="input" value={form.homeTeamId} onChange={(e) => setForm({ ...form, homeTeamId: Number(e.target.value) })}>
              {teams.map((t) => <option key={t.id} value={t.id}>{t.name}</option>)}
            </select>
          </div>
          <div>
            <label className="label">Away team</label>
            <select className="input" value={form.awayTeamId} onChange={(e) => setForm({ ...form, awayTeamId: Number(e.target.value) })}>
              {teams.map((t) => <option key={t.id} value={t.id}>{t.name}</option>)}
            </select>
          </div>
        </div>
        <div className="grid grid-cols-2 gap-3">
          <div>
            <label className="label">Stage</label>
            <select className="input" value={form.stage} onChange={(e) => setForm({ ...form, stage: e.target.value as Stage })}>
              {STAGES.map((s) => <option key={s} value={s}>{s}</option>)}
            </select>
          </div>
          <div>
            <label className="label">Date</label>
            <input className="input" type="date" value={form.date} onChange={(e) => setForm({ ...form, date: e.target.value })} />
          </div>
        </div>
        <div>
          <label className="label">Venue</label>
          <input className="input" value={form.venue} onChange={(e) => setForm({ ...form, venue: e.target.value })} />
        </div>
        <div className="flex gap-2">
          <button type="submit" className="btn-primary" data-action="save-match">{editing ? "Save" : "Schedule"}</button>
          {editing && (
            <button type="button" className="btn" onClick={() => { setEditing(null); setForm({ ...empty, homeTeamId: teams[0]?.id ?? 0, awayTeamId: teams[1]?.id ?? 0 }); }}>
              Cancel
            </button>
          )}
        </div>
      </form>
    </div>
  );
}
