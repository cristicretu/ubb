import { useEffect, useMemo, useState } from "react";
import { api, type Player, type Position, type Team } from "../api";

const POSITIONS: Position[] = ["GK", "DEF", "MID", "FWD"];

const empty: Omit<Player, "id"> = {
  teamId: 0,
  name: "",
  position: "FWD",
  jerseyNumber: 10,
  dateOfBirth: "2000-01-01",
  marketValueMillions: 10,
};

export default function PlayersPage() {
  const [players, setPlayers] = useState<Player[]>([]);
  const [teams, setTeams] = useState<Team[]>([]);
  const [filterTeamId, setFilterTeamId] = useState<number | "all">("all");
  const [editing, setEditing] = useState<Player | null>(null);
  const [form, setForm] = useState<Omit<Player, "id">>(empty);
  const [error, setError] = useState<string | null>(null);

  const refresh = async () => {
    const [ps, ts] = await Promise.all([api.listPlayers(), api.listTeams()]);
    setPlayers(ps);
    setTeams(ts);
    setForm((f) => (f.teamId === 0 && ts[0] ? { ...f, teamId: ts[0].id } : f));
  };
  useEffect(() => { refresh().catch((e) => setError(e.message)); }, []);

  const teamName = (id: number) => teams.find((t) => t.id === id)?.name ?? "?";

  const filtered = useMemo(
    () => (filterTeamId === "all" ? players : players.filter((p) => p.teamId === filterTeamId)),
    [players, filterTeamId],
  );

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    try {
      if (editing) await api.updatePlayer(editing.id, form);
      else await api.createPlayer(form);
      setForm({ ...empty, teamId: teams[0]?.id ?? 0 });
      setEditing(null);
      refresh();
    } catch (e: any) {
      setError(e.message);
    }
  };

  const remove = async (id: number) => {
    setError(null);
    try {
      await api.deletePlayer(id);
      refresh();
    } catch (e: any) {
      setError(e.message);
    }
  };

  const edit = (p: Player) => {
    setEditing(p);
    const { id, ...rest } = p;
    void id;
    setForm(rest);
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
      <div className="md:col-span-2 card">
        <div className="flex items-center justify-between mb-3">
          <h2 className="text-lg font-bold">Players</h2>
          <div className="flex items-center gap-2 text-sm">
            <label className="text-stone-600">Filter by team:</label>
            <select
              className="input w-44"
              value={filterTeamId}
              onChange={(e) =>
                setFilterTeamId(e.target.value === "all" ? "all" : Number(e.target.value))
              }
              data-testid="player-filter"
            >
              <option value="all">All teams</option>
              {teams.map((t) => <option key={t.id} value={t.id}>{t.name}</option>)}
            </select>
          </div>
        </div>
        <table className="w-full text-sm" data-testid="players-table">
          <thead>
            <tr className="text-left text-stone-500 border-b">
              <th className="py-1">#</th>
              <th>Name</th>
              <th>Team</th>
              <th>Pos</th>
              <th>Jersey</th>
              <th>DOB</th>
              <th>€M</th>
              <th></th>
            </tr>
          </thead>
          <tbody>
            {filtered.map((p) => (
              <tr key={p.id} className="border-b last:border-b-0">
                <td className="py-1.5 text-stone-500">{p.id}</td>
                <td className="font-medium">{p.name}</td>
                <td>{teamName(p.teamId)}</td>
                <td>{p.position}</td>
                <td>{p.jerseyNumber}</td>
                <td>{p.dateOfBirth}</td>
                <td>{p.marketValueMillions}</td>
                <td className="text-right space-x-1">
                  <button className="btn" onClick={() => edit(p)} data-action="edit-player">Edit</button>
                  <button className="btn-danger" onClick={() => remove(p.id)} data-action="delete-player">Delete</button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      <form className="card space-y-3" onSubmit={submit} data-testid="player-form">
        <h2 className="text-lg font-bold">{editing ? `Edit player #${editing.id}` : "Add new player"}</h2>
        {error && <div className="rounded bg-red-50 border border-red-200 px-2 py-1 text-sm text-red-700" data-testid="player-error">{error}</div>}
        <div>
          <label className="label">Name</label>
          <input className="input" value={form.name} onChange={(e) => setForm({ ...form, name: e.target.value })} />
        </div>
        <div>
          <label className="label">Team</label>
          <select className="input" value={form.teamId} onChange={(e) => setForm({ ...form, teamId: Number(e.target.value) })}>
            {teams.map((t) => <option key={t.id} value={t.id}>{t.name}</option>)}
          </select>
        </div>
        <div className="grid grid-cols-2 gap-3">
          <div>
            <label className="label">Position</label>
            <select className="input" value={form.position} onChange={(e) => setForm({ ...form, position: e.target.value as Position })}>
              {POSITIONS.map((p) => <option key={p} value={p}>{p}</option>)}
            </select>
          </div>
          <div>
            <label className="label">Jersey #</label>
            <input className="input" type="number" min={1} max={99} value={form.jerseyNumber} onChange={(e) => setForm({ ...form, jerseyNumber: Number(e.target.value) })} />
          </div>
        </div>
        <div>
          <label className="label">Date of birth</label>
          <input className="input" type="date" value={form.dateOfBirth} onChange={(e) => setForm({ ...form, dateOfBirth: e.target.value })} />
        </div>
        <div>
          <label className="label">Market value (€M)</label>
          <input className="input" type="number" min={0} max={500} step="0.5" value={form.marketValueMillions} onChange={(e) => setForm({ ...form, marketValueMillions: Number(e.target.value) })} />
        </div>
        <div className="flex gap-2">
          <button type="submit" className="btn-primary" data-action="save-player">{editing ? "Save" : "Create"}</button>
          {editing && (
            <button type="button" className="btn" onClick={() => { setEditing(null); setForm({ ...empty, teamId: teams[0]?.id ?? 0 }); }}>
              Cancel
            </button>
          )}
        </div>
      </form>
    </div>
  );
}
