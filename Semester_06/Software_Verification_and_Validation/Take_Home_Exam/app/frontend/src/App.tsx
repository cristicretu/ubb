import { NavLink, Navigate, Route, Routes } from "react-router-dom";
import TeamsPage from "./pages/TeamsPage";
import PlayersPage from "./pages/PlayersPage";
import MatchesPage from "./pages/MatchesPage";
import RecordResultPage from "./pages/RecordResultPage";
import TopScorersPage from "./pages/TopScorersPage";

const nav = [
  { to: "/teams", label: "Teams" },
  { to: "/players", label: "Players" },
  { to: "/matches", label: "Matches" },
  { to: "/record", label: "Record Match Result" },
  { to: "/top-scorers", label: "Top Scorers Report" },
];

export default function App() {
  return (
    <div className="min-h-screen flex flex-col">
      <header className="bg-pitch text-white">
        <div className="mx-auto max-w-6xl px-6 py-4 flex items-center justify-between">
          <div>
            <h1 className="text-xl font-bold tracking-tight">
              FIFA World Cup 2026 — Tracker
            </h1>
            <p className="text-emerald-100 text-xs">
              Group 933 · CRUD entities: Teams · Players · Matches
            </p>
          </div>
          <nav className="flex gap-1 text-sm">
            {nav.map((n) => (
              <NavLink
                key={n.to}
                to={n.to}
                className={({ isActive }) =>
                  "px-3 py-1.5 rounded-md " +
                  (isActive
                    ? "bg-emerald-800 text-whistle"
                    : "hover:bg-emerald-800/60 text-emerald-50")
                }
              >
                {n.label}
              </NavLink>
            ))}
          </nav>
        </div>
      </header>
      <main className="flex-1 mx-auto w-full max-w-6xl px-6 py-6">
        <Routes>
          <Route path="/" element={<Navigate to="/teams" replace />} />
          <Route path="/teams" element={<TeamsPage />} />
          <Route path="/players" element={<PlayersPage />} />
          <Route path="/matches" element={<MatchesPage />} />
          <Route path="/record" element={<RecordResultPage />} />
          <Route path="/top-scorers" element={<TopScorersPage />} />
        </Routes>
      </main>
      <footer className="border-t border-stone-200 py-3 text-center text-xs text-stone-500">
        SSVV Take-Home — Group 933 — Cretu · Grancea · Deaconu · Draguta
      </footer>
    </div>
  );
}
