"use client";

import { useState } from "react";
import Image from "next/image";

// Candidate interface
interface Candidate {
  id: number;
  name: string;
  image: string;
  party: string;
  description: string;
}

// In-memory data of popular Romanian political candidates
const initialCandidates: Candidate[] = [
  {
    id: 1,
    name: "Nicușor Dan",
    image: "https://placehold.co/600x400/000000/FFFFFF.png?text=ND",
    party: "Independent",
    description:
      "Current Mayor of Bucharest and the new President of Romania. Mathematics PhD, civic activist turned politician. Founded 'Save Bucharest' Association and later Save Romania Union (USR). Known for anti-corruption stance and pro-European policies.",
  },
  {
    id: 2,
    name: "Marcel Ciolacu",
    image: "https://placehold.co/600x400/000000/FFFFFF.png?text=MC",
    party: "PSD (Social Democratic Party)",
    description:
      "Former Prime Minister of Romania and current President of PSD. Previously served as President of the Chamber of Deputies. Known for his pragmatic approach and social democratic policies.",
  },
  {
    id: 3,
    name: "Elena Lasconi",
    image: "https://placehold.co/600x400/000000/FFFFFF.png?text=EL",
    party: "USR (Save Romania Union)",
    description:
      "Mayor of Câmpulung and former USR presidential candidate. Economist and journalist by background. Advocates for reformist policies, European integration, and anti-corruption measures.",
  },
  {
    id: 4,
    name: "George Simion",
    image: "https://placehold.co/600x400/000000/FFFFFF.png?text=GS",
    party: "AUR (Alliance for the Union of Romanians)",
    description:
      "Leader of the nationalist AUR party and Deputy in the Romanian Parliament. Civic activist and politician known for his nationalist and traditionalist positions.",
  },
  {
    id: 5,
    name: "Nicolae Ciucă",
    image: "https://placehold.co/600x400/000000/FFFFFF.png?text=NC",
    party: "PNL (National Liberal Party)",
    description:
      "Former Prime Minister and current President of the Senate. Former Chief of the Romanian General Staff and military officer. Represents center-right liberal politics.",
  },
  {
    id: 6,
    name: "Mircea Geoană",
    image: "https://placehold.co/600x400/000000/FFFFFF.png?text=MG",
    party: "Independent",
    description:
      "Former Deputy Secretary General of NATO and former President of PSD. Previously served as Minister of Foreign Affairs and Senator. Experienced diplomat with international relations background.",
  },
  {
    id: 7,
    name: "Călin Georgescu",
    image: "https://placehold.co/600x400/000000/FFFFFF.png?text=CG",
    party: "Independent",
    description:
      "Former UN executive director and agronomist. Surprised in the 2024 presidential elections before they were annulled. Known for nationalist positions and social media campaigning.",
  },
  {
    id: 8,
    name: "Klaus Iohannis",
    image: "https://placehold.co/600x400/000000/FFFFFF.png?text=KI",
    party: "Independent (formerly PNL)",
    description:
      "Former President of Romania (2014-2025). Former mayor of Sibiu and physics teacher. Known for his pro-European stance and institutional reforms during his presidency.",
  },
];

export default function Home() {
  const [candidates, setCandidates] = useState<Candidate[]>(initialCandidates);
  const [selectedCandidate, setSelectedCandidate] = useState<Candidate | null>(
    null,
  );
  const [searchTerm, setSearchTerm] = useState("");

  // Filter candidates based on search term
  const filteredCandidates = candidates.filter(
    (candidate) =>
      candidate.name.toLowerCase().includes(searchTerm.toLowerCase()) ||
      candidate.party.toLowerCase().includes(searchTerm.toLowerCase()),
  );

  return (
    <main className="min-h-screen bg-gradient-to-br from-blue-50 to-red-50 p-4">
      <div className="container mx-auto max-w-6xl">
        {/* Header */}
        <div className="mb-8 text-center">
          <h1 className="mb-4 text-4xl font-bold text-gray-800">
            🇷🇴 Romanian Political Candidates
          </h1>
          <p className="text-lg text-gray-600">
            Explore the profiles of prominent Romanian political figures
          </p>
        </div>

        {/* Search Bar */}
        <div className="mb-8">
          <input
            type="text"
            placeholder="Search by name or party..."
            value={searchTerm}
            onChange={(e) => setSearchTerm(e.target.value)}
            className="mx-auto block w-full max-w-md rounded-lg border border-gray-300 px-4 py-2 focus:border-transparent focus:ring-2 focus:ring-blue-500"
          />
        </div>

        {/* Candidates Grid */}
        <div className="mb-8 grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
          {filteredCandidates.map((candidate) => (
            <div
              key={candidate.id}
              className="cursor-pointer rounded-lg border border-gray-200 bg-white shadow-md transition-shadow hover:shadow-lg"
              onClick={() => setSelectedCandidate(candidate)}
            >
              <div className="p-6">
                {/* Candidate image */}
                <div className="mx-auto mb-4 h-24 w-24 overflow-hidden rounded-full">
                  <Image
                    src={candidate.image}
                    alt={candidate.name}
                    width={96}
                    height={96}
                    className="h-full w-full object-cover"
                  />
                </div>
                <h3 className="mb-2 text-center text-xl font-semibold text-gray-800">
                  {candidate.name}
                </h3>
                <p className="mb-3 text-center text-sm font-medium text-blue-600">
                  {candidate.party}
                </p>
                <p className="line-clamp-3 text-sm text-gray-600">
                  {candidate.description.slice(0, 100)}...
                </p>
                <button className="mt-4 w-full rounded bg-blue-500 px-4 py-2 text-white transition-colors hover:bg-blue-600">
                  View Details
                </button>
              </div>
            </div>
          ))}
        </div>

        {/* No results message */}
        {filteredCandidates.length === 0 && (
          <div className="py-8 text-center">
            <p className="text-lg text-gray-500">
              No candidates found matching your search.
            </p>
          </div>
        )}

        {/* Modal for candidate details */}
        {selectedCandidate && (
          <div className="bg-opacity-50 fixed inset-0 z-50 flex items-center justify-center bg-black p-4">
            <div className="max-h-[90vh] w-full max-w-2xl overflow-y-auto rounded-lg bg-white">
              <div className="p-6">
                <div className="mb-4 flex items-start justify-between">
                  <h2 className="text-2xl font-bold text-gray-800">
                    {selectedCandidate.name}
                  </h2>
                  <button
                    onClick={() => setSelectedCandidate(null)}
                    className="text-2xl text-gray-500 hover:text-gray-700"
                  >
                    ×
                  </button>
                </div>

                <div className="mb-6">
                  {/* Candidate image */}
                  <div className="mx-auto mb-4 h-32 w-32 overflow-hidden rounded-full">
                    <Image
                      src={selectedCandidate.image}
                      alt={selectedCandidate.name}
                      width={128}
                      height={128}
                      className="h-full w-full object-cover"
                    />
                  </div>
                </div>

                <div className="space-y-4">
                  <div>
                    <h3 className="font-semibold text-gray-800">Party:</h3>
                    <p className="text-blue-600">{selectedCandidate.party}</p>
                  </div>

                  <div>
                    <h3 className="font-semibold text-gray-800">
                      Description:
                    </h3>
                    <p className="leading-relaxed text-gray-600">
                      {selectedCandidate.description}
                    </p>
                  </div>
                </div>

                <div className="mt-6 flex justify-end">
                  <button
                    onClick={() => setSelectedCandidate(null)}
                    className="rounded bg-gray-500 px-6 py-2 text-white transition-colors hover:bg-gray-600"
                  >
                    Close
                  </button>
                </div>
              </div>
            </div>
          </div>
        )}

        {/* Stats */}
        <div className="mt-8 rounded-lg bg-white p-6 shadow-md">
          <h2 className="mb-4 text-xl font-semibold text-gray-800">
            Quick Stats
          </h2>
          <div className="grid grid-cols-2 gap-4 text-center md:grid-cols-4">
            <div>
              <div className="text-2xl font-bold text-blue-600">
                {candidates.length}
              </div>
              <div className="text-sm text-gray-600">Total Candidates</div>
            </div>
            <div>
              <div className="text-2xl font-bold text-green-600">
                {new Set(candidates.map((c) => c.party)).size}
              </div>
              <div className="text-sm text-gray-600">Political Parties</div>
            </div>
            <div>
              <div className="text-2xl font-bold text-purple-600">
                {candidates.filter((c) => c.party === "Independent").length}
              </div>
              <div className="text-sm text-gray-600">Independent</div>
            </div>
            <div>
              <div className="text-2xl font-bold text-red-600">
                {filteredCandidates.length}
              </div>
              <div className="text-sm text-gray-600">Search Results</div>
            </div>
          </div>
        </div>
      </div>
    </main>
  );
}
