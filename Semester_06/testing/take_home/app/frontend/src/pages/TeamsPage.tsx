import { useEffect, useState } from "react";
import { api, type Team, type GroupLetter } from "../api";

const GROUPS: GroupLetter[] = [
  "A","B","C","D","E","F","G","H","I","J","K","L",
];

const empty: Omit<Team, "id"> = {
  name: "", country: "", group: "A", coach: "", fifaRanking: 50,
};

export default function TeamsPage() {
  const [teams, setTeams] = useState<Team[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [editing, setEditing] = useState<Team | null>(null);
  const [form, setForm] = useState<Omit<Team, "id">>(empty);

  const refresh = () => api.listTeams().then(setTeams).catch((e) => setError(e.message));
  useEffect(() => { refresh(); }, []);

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    try {
      if (editing) {
        await api.updateTeam(editing.id, form);
      } else {
        await api.createTeam(form);
      }
      setForm(empty);
      setEditing(null);
      refresh();
    } catch (e: any) {
      setError(e.message);
    }
  };

  const remove = async (id: number) => {
    setError(null);
    try {
      await api.deleteTeam(id);
      refresh();
    } catch (e: any) {
      setError(e.message);
    }
  };

  const edit = (t: Team) => {
    setEditing(t);
    const { id, ...rest } = t;
    void id;
    setForm(rest);
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
      <div className="md:col-span-2 card">
        <h2 className="text-lg font-bold mb-3">Teams</h2>
        <table className="w-full text-sm" data-testid="teams-table">
          <thead>
            <tr className="text-left text-stone-500 border-b">
              <th className="py-1">#</th>
              <th>Name</th>
              <th>Country</th>
              <th>Group</th>
              <th>Coach</th>
              <th>FIFA</th>
              <th></th>
            </tr>
          </thead>
          <tbody>
            {teams.map((t) => (
              <tr key={t.id} className="border-b last:border-b-0">
                <td className="py-1.5 text-stone-500">{t.id}</td>
                <td className="font-medium">{t.name}</td>
                <td>{t.country}</td>
                <td>
                  <span className="rounded bg-emerald-100 px-1.5 py-0.5 text-xs font-semibold text-emerald-800">
                    {t.group}
                  </span>
                </td>
                <td>{t.coach}</td>
                <td>{t.fifaRanking}</td>
                <td className="text-right space-x-1">
                  <button className="btn" onClick={() => edit(t)} data-action="edit-team">Edit</button>
                  <button className="btn-danger" onClick={() => remove(t.id)} data-action="delete-team">Delete</button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      <form className="card space-y-3" onSubmit={submit} data-testid="team-form">
        <h2 className="text-lg font-bold">{editing ? `Edit team #${editing.id}` : "Add new team"}</h2>
        {error && <div className="rounded bg-red-50 border border-red-200 px-2 py-1 text-sm text-red-700" data-testid="team-error">{error}</div>}
        <div>
          <label className="label">Name</label>
          <input className="input" name="name" value={form.name} onChange={(e) => setForm({ ...form, name: e.target.value })} />
        </div>
        <div>
          <label className="label">Country</label>
          <input className="input" name="country" value={form.country} onChange={(e) => setForm({ ...form, country: e.target.value })} />
        </div>
        <div className="grid grid-cols-2 gap-3">
          <div>
            <label className="label">Group</label>
            <select className="input" name="group" value={form.group} onChange={(e) => setForm({ ...form, group: e.target.value as GroupLetter })}>
              {GROUPS.map((g) => <option key={g} value={g}>{g}</option>)}
            </select>
          </div>
          <div>
            <label className="label">FIFA rank</label>
            <input className="input" type="number" name="fifaRanking" min={1} max={210}
              value={form.fifaRanking}
              onChange={(e) => setForm({ ...form, fifaRanking: Number(e.target.value) })} />
          </div>
        </div>
        <div>
          <label className="label">Coach</label>
          <input className="input" name="coach" value={form.coach} onChange={(e) => setForm({ ...form, coach: e.target.value })} />
        </div>
        <div className="flex gap-2">
          <button type="submit" className="btn-primary" data-action="save-team">{editing ? "Save" : "Create"}</button>
          {editing && (
            <button type="button" className="btn" onClick={() => { setEditing(null); setForm(empty); }}>
              Cancel
            </button>
          )}
        </div>
      </form>
    </div>
  );
}
