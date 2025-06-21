"use client";

import { useState, useRef, useEffect } from "react";
import Image from "next/image";
import { Button } from "~/components/ui/button";
import {
  Card,
  CardContent,
  CardDescription,
  CardFooter,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import { Input } from "~/components/ui/input";
import { Textarea } from "~/components/ui/textarea";
import { Badge } from "~/components/ui/badge";
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
} from "~/components/ui/dialog";
import { Alert, AlertDescription } from "~/components/ui/alert";
import type { ChartConfig } from "~/components/ui/chart";
import {
  ChartContainer,
  ChartTooltip,
  ChartTooltipContent,
} from "~/components/ui/chart";
import {
  Search,
  Plus,
  Eye,
  Edit2,
  Trash2,
  Users,
  Building2,
  UserCheck,
  BarChart3,
  Play,
  Square,
} from "lucide-react";
import {
  BarChart,
  Bar,
  CartesianGrid,
  XAxis,
  YAxis,
  ResponsiveContainer,
  Cell,
} from "recharts";

// Candidate interface
interface Candidate {
  id: number;
  name: string;
  image: string;
  party: string;
  description: string;
}

interface CandidateFormData {
  name: string;
  image: string;
  party: string;
  description: string;
}

interface ChartDataPoint {
  party: string;
  count: number;
  color: string;
}

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

// Random name generators
const firstNames = [
  "Alexandru",
  "Andrei",
  "Adrian",
  "Bogdan",
  "Cristian",
  "Dan",
  "Emil",
  "Florin",
  "Gabriel",
  "Ion",
  "Lucian",
  "Marius",
  "Mihai",
  "Nicolae",
  "Paul",
  "Radu",
  "Stefan",
  "Victor",
  "Vlad",
  "Gheorghe",
  "Ana",
  "Elena",
  "Maria",
  "Ioana",
  "Cristina",
  "Diana",
  "Laura",
  "Monica",
  "Andreea",
  "Raluca",
];

const lastNames = [
  "Popescu",
  "Ionescu",
  "Popa",
  "Stoica",
  "Dumitrescu",
  "Georgescu",
  "Stanciu",
  "Munteanu",
  "Rusu",
  "Preda",
  "Constantinescu",
  "Moldovan",
  "Petrescu",
  "Nicolae",
  "Barbu",
  "Cristea",
  "Florea",
  "Vasile",
  "Tudor",
  "Matei",
];

const parties = [
  "PSD (Social Democratic Party)",
  "USR (Save Romania Union)",
  "AUR (Alliance for the Union of Romanians)",
  "PNL (National Liberal Party)",
  "Independent",
];

const descriptions = [
  "Experienced politician with a background in public administration and economic development.",
  "Former mayor known for transparency initiatives and urban development projects.",
  "Academic turned politician, advocating for educational reform and digital transformation.",
  "Business leader focusing on entrepreneurship and economic growth policies.",
  "Civil rights activist promoting social justice and anti-corruption measures.",
  "Former diplomat with extensive experience in international relations and EU affairs.",
  "Young politician advocating for climate action and sustainable development.",
  "Legal expert specializing in constitutional law and judicial reform.",
  "Healthcare professional promoting public health initiatives and medical system reform.",
  "Technology entrepreneur focused on digital innovation and startup ecosystem development.",
];

const partyColors = {
  PSD: "#dc2626", // Red
  USR: "#2563eb", // Blue
  AUR: "#ea580c", // Orange
  PNL: "#facc15", // Yellow
  Independent: "#6b7280", // Gray
};

const chartConfig = {
  count: {
    label: "Candidates",
    color: "hsl(var(--chart-1))",
  },
} satisfies ChartConfig;

export default function Home() {
  const [candidates, setCandidates] = useState<Candidate[]>(initialCandidates);
  const [selectedCandidate, setSelectedCandidate] = useState<Candidate | null>(
    null,
  );
  const [searchTerm, setSearchTerm] = useState("");
  const [showForm, setShowForm] = useState(false);
  const [editingCandidate, setEditingCandidate] = useState<Candidate | null>(
    null,
  );
  const [deleteConfirmation, setDeleteConfirmation] =
    useState<Candidate | null>(null);
  const [message, setMessage] = useState<{
    type: "success" | "error";
    text: string;
  } | null>(null);

  const [formData, setFormData] = useState<CandidateFormData>({
    name: "",
    image: "",
    party: "",
    description: "",
  });

  const [formErrors, setFormErrors] = useState<Partial<CandidateFormData>>({});

  // Chart and generation state
  const [chartData, setChartData] = useState<ChartDataPoint[]>([]);
  const [isGenerating, setIsGenerating] = useState(false);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);

  // Helper function to get party counts
  const getPartyCounts = (candidateList: Candidate[]): ChartDataPoint[] => {
    const counts: Record<string, number> = {
      PSD: 0,
      USR: 0,
      AUR: 0,
      PNL: 0,
      Independent: 0,
    };

    candidateList.forEach((candidate) => {
      if (candidate.party.includes("PSD")) counts.PSD++;
      else if (candidate.party.includes("USR")) counts.USR++;
      else if (candidate.party.includes("AUR")) counts.AUR++;
      else if (candidate.party.includes("PNL")) counts.PNL++;
      else counts.Independent++;
    });

    return Object.entries(counts).map(([party, count]) => ({
      party: party === "Independent" ? "Independent" : party,
      count,
      color: partyColors[party as keyof typeof partyColors],
    }));
  };

  // Initialize chart data
  useEffect(() => {
    setChartData(getPartyCounts(candidates));
  }, []);

  // Generate random candidate
  const generateRandomCandidate = (): Candidate => {
    const firstName =
      firstNames[Math.floor(Math.random() * firstNames.length)]!;
    const lastName = lastNames[Math.floor(Math.random() * lastNames.length)]!;
    const party = parties[Math.floor(Math.random() * parties.length)]!;
    const description =
      descriptions[Math.floor(Math.random() * descriptions.length)]!;

    const initials = firstName.charAt(0) + lastName.charAt(0);

    return {
      id: Date.now() + Math.random(),
      name: `${firstName} ${lastName}`,
      image: `https://placehold.co/600x400/000000/FFFFFF.png?text=${initials}`,
      party,
      description,
    };
  };

  // Start generation
  const startGeneration = () => {
    if (intervalRef.current) return;

    setIsGenerating(true);
    intervalRef.current = setInterval(() => {
      const newCandidate = generateRandomCandidate();
      setCandidates((prev) => {
        const updated = [...prev, newCandidate];
        // Update chart data
        setChartData(getPartyCounts(updated));
        return updated;
      });
    }, 2000); // Add new candidate every 2 seconds
  };

  // Stop generation
  const stopGeneration = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
      setIsGenerating(false);
    }
  };

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, []);

  const filteredCandidates = candidates.filter(
    (candidate) =>
      candidate.name.toLowerCase().includes(searchTerm.toLowerCase()) ||
      candidate.party.toLowerCase().includes(searchTerm.toLowerCase()),
  );

  const validateForm = (
    data: CandidateFormData,
  ): Partial<CandidateFormData> => {
    const errors: Partial<CandidateFormData> = {};

    if (!data.name.trim()) {
      errors.name = "Name is required";
    } else if (data.name.trim().length < 2) {
      errors.name = "Name must be at least 2 characters";
    }

    if (!data.party.trim()) {
      errors.party = "Party is required";
    }

    if (!data.description.trim()) {
      errors.description = "Description is required";
    } else if (data.description.trim().length < 10) {
      errors.description = "Description must be at least 10 characters";
    }

    if (!data.image.trim()) {
      errors.image = "Image URL is required";
    } else {
      try {
        new URL(data.image);
      } catch {
        errors.image = "Please enter a valid URL";
      }
    }

    return errors;
  };

  const showMessage = (type: "success" | "error", text: string) => {
    setMessage({ type, text });
    setTimeout(() => setMessage(null), 3000);
  };

  const handleCreate = () => {
    const errors = validateForm(formData);
    setFormErrors(errors);

    if (Object.keys(errors).length > 0) {
      showMessage("error", "Please fix the validation errors");
      return;
    }

    if (
      candidates.some(
        (c) => c.name.toLowerCase() === formData.name.toLowerCase(),
      )
    ) {
      setFormErrors({ name: "A candidate with this name already exists" });
      showMessage("error", "Candidate name already exists");
      return;
    }

    const newCandidate: Candidate = {
      id: Math.max(...candidates.map((c) => c.id), 0) + 1,
      ...formData,
    };

    setCandidates((prev) => {
      const updated = [...prev, newCandidate];
      // Update chart data
      setChartData(getPartyCounts(updated));
      return updated;
    });
    resetForm();
    showMessage("success", `${newCandidate.name} has been added successfully!`);
  };

  const handleUpdate = () => {
    if (!editingCandidate) return;

    const errors = validateForm(formData);
    setFormErrors(errors);

    if (Object.keys(errors).length > 0) {
      showMessage("error", "Please fix the validation errors");
      return;
    }

    if (
      candidates.some(
        (c) =>
          c.id !== editingCandidate.id &&
          c.name.toLowerCase() === formData.name.toLowerCase(),
      )
    ) {
      setFormErrors({ name: "A candidate with this name already exists" });
      showMessage("error", "Candidate name already exists");
      return;
    }

    setCandidates((prev) => {
      const updated = prev.map((candidate) =>
        candidate.id === editingCandidate.id
          ? { ...candidate, ...formData }
          : candidate,
      );
      // Update chart data
      setChartData(getPartyCounts(updated));
      return updated;
    });

    resetForm();
    showMessage("success", `${formData.name} has been updated successfully!`);
  };

  const handleDelete = (candidate: Candidate) => {
    setCandidates((prev) => {
      const updated = prev.filter((c) => c.id !== candidate.id);
      // Update chart data
      setChartData(getPartyCounts(updated));
      return updated;
    });
    setDeleteConfirmation(null);
    setSelectedCandidate(null);
    showMessage("success", `${candidate.name} has been deleted successfully!`);
  };

  const resetForm = () => {
    setFormData({
      name: "",
      image: "",
      party: "",
      description: "",
    });
    setFormErrors({});
    setShowForm(false);
    setEditingCandidate(null);
  };

  const startEdit = (candidate: Candidate) => {
    setEditingCandidate(candidate);
    setFormData({
      name: candidate.name,
      image: candidate.image,
      party: candidate.party,
      description: candidate.description,
    });
    setFormErrors({});
    setShowForm(true);
    setSelectedCandidate(null);
  };

  // Handle form input changes
  const handleInputChange = (field: keyof CandidateFormData, value: string) => {
    setFormData((prev) => ({ ...prev, [field]: value }));
    // Clear error for this field when user starts typing
    if (formErrors[field]) {
      setFormErrors((prev) => ({ ...prev, [field]: undefined }));
    }
  };

  return (
    <main className="min-h-screen bg-slate-50 py-8">
      <div className="container mx-auto max-w-7xl px-4">
        {/* Header */}
        <div className="mb-8">
          <h1 className="mb-2 text-3xl font-bold text-slate-900">
            Romanian Political Candidates
          </h1>
          <p className="text-lg text-slate-600">
            Explore and manage profiles of prominent Romanian political figures
          </p>
        </div>

        {/* Real-time Chart */}
        <Card className="mb-8 border-0 bg-gradient-to-br from-slate-50 to-white shadow-lg">
          <CardHeader className="pb-4">
            <div className="flex items-center justify-between">
              <div>
                <CardTitle className="flex items-center gap-3 text-2xl font-bold text-slate-800">
                  <div className="rounded-lg bg-blue-100 p-2">
                    <BarChart3 className="h-6 w-6 text-blue-600" />
                  </div>
                  Candidate Distribution by Party
                </CardTitle>
                <CardDescription className="mt-2 text-base text-slate-600">
                  Live tracking of candidates across political parties
                  {isGenerating && (
                    <span className="ml-2 inline-flex items-center gap-1 text-green-600">
                      <div className="h-2 w-2 animate-pulse rounded-full bg-green-500"></div>
                      Auto-generating
                    </span>
                  )}
                </CardDescription>
              </div>
              <div className="flex gap-3">
                <Button
                  onClick={startGeneration}
                  disabled={isGenerating}
                  className="flex items-center gap-2 bg-green-600 shadow-md hover:bg-green-700"
                  size="lg"
                >
                  <Play className="h-4 w-4" />
                  {isGenerating ? "Generating..." : "Start Auto-Gen"}
                </Button>
                <Button
                  onClick={stopGeneration}
                  disabled={!isGenerating}
                  variant="destructive"
                  className="flex items-center gap-2 shadow-md"
                  size="lg"
                >
                  <Square className="h-4 w-4" />
                  Stop
                </Button>
              </div>
            </div>
          </CardHeader>
          <CardContent className="pt-0">
            <div className="rounded-xl bg-white p-6 shadow-inner">
              <ChartContainer
                config={chartConfig}
                className="min-h-[400px] w-full"
              >
                <ResponsiveContainer width="100%" height={400}>
                  <BarChart
                    data={chartData}
                    margin={{ top: 20, right: 30, left: 20, bottom: 60 }}
                    barCategoryGap="20%"
                  >
                    <CartesianGrid strokeDasharray="3 3" stroke="#e2e8f0" />
                    <XAxis
                      dataKey="party"
                      tick={{ fontSize: 12, fontWeight: 600 }}
                      angle={-45}
                      textAnchor="end"
                      height={60}
                      interval={0}
                    />
                    <YAxis
                      tick={{ fontSize: 12 }}
                      label={{
                        value: "Number of Candidates",
                        angle: -90,
                        position: "insideLeft",
                      }}
                    />
                    <ChartTooltip
                      content={({ active, payload, label }) => {
                        if (active && payload && payload.length) {
                          return (
                            <div className="rounded-lg border border-slate-200 bg-white p-3 shadow-lg">
                              <p className="font-semibold text-slate-800">
                                {label}
                              </p>
                              <p className="text-blue-600">
                                Candidates:{" "}
                                <span className="font-bold">
                                  {payload[0]?.value}
                                </span>
                              </p>
                            </div>
                          );
                        }
                        return null;
                      }}
                    />
                    <Bar dataKey="count" radius={[4, 4, 0, 0]}>
                      {chartData.map((entry, index) => (
                        <Cell key={`cell-${index}`} fill={entry.color} />
                      ))}
                    </Bar>
                  </BarChart>
                </ResponsiveContainer>
              </ChartContainer>
            </div>

            {/* Party Legend */}
            <div className="mt-6 flex flex-wrap justify-center gap-4">
              {chartData.map((item, index) => (
                <div
                  key={index}
                  className="flex items-center gap-2 rounded-lg bg-slate-50 px-3 py-2"
                >
                  <div
                    className="h-3 w-3 rounded-full"
                    style={{ backgroundColor: item.color }}
                  ></div>
                  <span className="text-sm font-medium text-slate-700">
                    {item.party === "Independent" ? "Independent" : item.party}
                  </span>
                  <span className="text-sm font-bold text-slate-900">
                    ({item.count})
                  </span>
                </div>
              ))}
            </div>
          </CardContent>
        </Card>

        {/* Success/Error Message */}
        {message && (
          <div
            className={`mb-6 rounded-lg p-4 text-center ${
              message.type === "success"
                ? "border border-green-400 bg-green-100 text-green-700"
                : "border border-red-400 bg-red-100 text-red-700"
            }`}
          >
            {message.text}
          </div>
        )}

        {/* Search Bar and Add Button */}
        <div className="mb-8 flex flex-col gap-4 md:flex-row md:items-center md:justify-between">
          <input
            type="text"
            placeholder="Search by name or party..."
            value={searchTerm}
            onChange={(e) => setSearchTerm(e.target.value)}
            className="w-full max-w-md rounded-lg border border-gray-300 px-4 py-2 focus:border-transparent focus:ring-2 focus:ring-blue-500"
          />
          <button
            onClick={() => {
              setEditingCandidate(null);
              setFormData({
                name: "",
                image: "",
                party: "",
                description: "",
              });
              setFormErrors({});
              setShowForm(true);
            }}
            className="min-w-fit transform rounded-lg bg-gradient-to-r from-green-500 to-green-600 px-8 py-3 font-semibold text-white shadow-lg transition-all duration-200 hover:scale-105 hover:from-green-600 hover:to-green-700 hover:shadow-xl"
          >
            ✨ Add New Candidate
          </button>
        </div>

        {/* Candidates Grid */}
        <div className="mb-8 grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
          {filteredCandidates.map((candidate) => (
            <div
              key={candidate.id}
              className="rounded-lg border border-gray-200 bg-white shadow-md transition-shadow hover:shadow-lg"
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

                {/* Action buttons */}
                <div className="mt-6 space-y-2">
                  <button
                    onClick={() => setSelectedCandidate(candidate)}
                    className="w-full rounded-lg bg-blue-500 px-4 py-2.5 text-sm font-medium text-white transition-all duration-200 hover:bg-blue-600 hover:shadow-md"
                  >
                    👁️ View Details
                  </button>
                  <div className="flex gap-2">
                    <button
                      onClick={() => startEdit(candidate)}
                      className="flex-1 rounded-lg bg-orange-500 px-3 py-2 text-sm font-medium text-white transition-all duration-200 hover:bg-orange-600 hover:shadow-md"
                    >
                      ✏️ Edit
                    </button>
                    <button
                      onClick={() => setDeleteConfirmation(candidate)}
                      className="flex-1 rounded-lg bg-red-500 px-3 py-2 text-sm font-medium text-white transition-all duration-200 hover:bg-red-600 hover:shadow-md"
                    >
                      🗑️ Delete
                    </button>
                  </div>
                </div>
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

        {/* Create/Edit Form Modal */}
        {showForm && (
          <div className="bg-opacity-60 fixed inset-0 z-50 flex items-center justify-center bg-black p-4 backdrop-blur-sm">
            <div className="max-h-[90vh] w-full max-w-2xl overflow-y-auto rounded-xl bg-white shadow-2xl">
              <div className="p-6">
                <div className="mb-6 flex items-start justify-between">
                  <h2 className="text-3xl font-bold text-gray-800">
                    {editingCandidate
                      ? "✏️ Edit Candidate"
                      : "✨ Add New Candidate"}
                  </h2>
                  <button
                    onClick={resetForm}
                    className="rounded-full p-2 text-2xl text-gray-500 transition-colors hover:bg-gray-100 hover:text-gray-700"
                  >
                    ×
                  </button>
                </div>

                <form
                  className="space-y-5"
                  onSubmit={(e) => e.preventDefault()}
                >
                  {/* Name field */}
                  <div>
                    <label className="mb-2 block text-sm font-semibold text-gray-700">
                      Full Name *
                    </label>
                    <input
                      type="text"
                      value={formData.name}
                      onChange={(e) =>
                        handleInputChange("name", e.target.value)
                      }
                      className={`w-full rounded-lg border-2 px-4 py-3 shadow-sm transition-all focus:border-blue-500 focus:ring-2 focus:ring-blue-200 ${
                        formErrors.name
                          ? "border-red-500 bg-red-50"
                          : "border-gray-300"
                      }`}
                      placeholder="Enter candidate's full name"
                    />
                    {formErrors.name && (
                      <p className="mt-2 text-sm font-medium text-red-600">
                        ⚠️ {formErrors.name}
                      </p>
                    )}
                  </div>

                  {/* Party field */}
                  <div>
                    <label className="mb-2 block text-sm font-semibold text-gray-700">
                      Political Party *
                    </label>
                    <input
                      type="text"
                      value={formData.party}
                      onChange={(e) =>
                        handleInputChange("party", e.target.value)
                      }
                      className={`w-full rounded-lg border-2 px-4 py-3 shadow-sm transition-all focus:border-blue-500 focus:ring-2 focus:ring-blue-200 ${
                        formErrors.party
                          ? "border-red-500 bg-red-50"
                          : "border-gray-300"
                      }`}
                      placeholder="Enter party name or Independent"
                    />
                    {formErrors.party && (
                      <p className="mt-2 text-sm font-medium text-red-600">
                        ⚠️ {formErrors.party}
                      </p>
                    )}
                  </div>

                  {/* Image URL field */}
                  <div>
                    <label className="mb-2 block text-sm font-semibold text-gray-700">
                      Profile Image URL *
                    </label>
                    <input
                      type="url"
                      value={formData.image}
                      onChange={(e) =>
                        handleInputChange("image", e.target.value)
                      }
                      className={`w-full rounded-lg border-2 px-4 py-3 shadow-sm transition-all focus:border-blue-500 focus:ring-2 focus:ring-blue-200 ${
                        formErrors.image
                          ? "border-red-500 bg-red-50"
                          : "border-gray-300"
                      }`}
                      placeholder="https://example.com/image.jpg"
                    />
                    {formErrors.image && (
                      <p className="mt-2 text-sm font-medium text-red-600">
                        ⚠️ {formErrors.image}
                      </p>
                    )}
                  </div>

                  {/* Description field */}
                  <div>
                    <label className="mb-2 block text-sm font-semibold text-gray-700">
                      Biography & Description *
                    </label>
                    <textarea
                      value={formData.description}
                      onChange={(e) =>
                        handleInputChange("description", e.target.value)
                      }
                      rows={5}
                      className={`w-full resize-none rounded-lg border-2 px-4 py-3 shadow-sm transition-all focus:border-blue-500 focus:ring-2 focus:ring-blue-200 ${
                        formErrors.description
                          ? "border-red-500 bg-red-50"
                          : "border-gray-300"
                      }`}
                      placeholder="Enter detailed description about the candidate's background, achievements, and political positions..."
                    />
                    {formErrors.description && (
                      <p className="mt-2 text-sm font-medium text-red-600">
                        ⚠️ {formErrors.description}
                      </p>
                    )}
                  </div>

                  {/* Form actions */}
                  <div className="flex justify-end gap-4 border-t pt-6">
                    <button
                      type="button"
                      onClick={resetForm}
                      className="rounded-lg bg-gray-500 px-8 py-3 font-semibold text-white transition-all duration-200 hover:bg-gray-600 hover:shadow-md"
                    >
                      Cancel
                    </button>
                    <button
                      type="button"
                      onClick={editingCandidate ? handleUpdate : handleCreate}
                      className="rounded-lg bg-blue-500 px-8 py-3 font-semibold text-white transition-all duration-200 hover:bg-blue-600 hover:shadow-md"
                    >
                      {editingCandidate
                        ? "💾 Update Candidate"
                        : "✨ Create Candidate"}
                    </button>
                  </div>
                </form>
              </div>
            </div>
          </div>
        )}

        {/* Delete Confirmation Modal */}
        {deleteConfirmation && (
          <div className="bg-opacity-60 fixed inset-0 z-50 flex items-center justify-center bg-black p-4 backdrop-blur-sm">
            <div className="w-full max-w-md rounded-xl bg-white p-8 shadow-2xl">
              <div className="text-center">
                <div className="mx-auto mb-4 flex h-16 w-16 items-center justify-center rounded-full bg-red-100">
                  <span className="text-2xl">🗑️</span>
                </div>
                <h2 className="mb-4 text-2xl font-bold text-gray-800">
                  Confirm Delete
                </h2>
                <p className="mb-8 leading-relaxed text-gray-600">
                  Are you sure you want to permanently delete{" "}
                  <strong className="text-red-600">
                    {deleteConfirmation.name}
                  </strong>
                  ?
                  <br />
                  <span className="text-sm">This action cannot be undone.</span>
                </p>
              </div>
              <div className="flex justify-center gap-4">
                <button
                  onClick={() => setDeleteConfirmation(null)}
                  className="rounded-lg bg-gray-500 px-6 py-3 font-semibold text-white transition-all duration-200 hover:bg-gray-600 hover:shadow-md"
                >
                  Cancel
                </button>
                <button
                  onClick={() => handleDelete(deleteConfirmation)}
                  className="rounded-lg bg-red-500 px-6 py-3 font-semibold text-white transition-all duration-200 hover:bg-red-600 hover:shadow-md"
                >
                  🗑️ Delete
                </button>
              </div>
            </div>
          </div>
        )}

        {/* View Details Modal */}
        {selectedCandidate && (
          <div className="bg-opacity-60 fixed inset-0 z-50 flex items-center justify-center bg-black p-4 backdrop-blur-sm">
            <div className="max-h-[90vh] w-full max-w-2xl overflow-y-auto rounded-xl bg-white shadow-2xl">
              <div className="p-8">
                <div className="mb-6 flex items-start justify-between">
                  <h2 className="text-3xl font-bold text-gray-800">
                    👤 {selectedCandidate.name}
                  </h2>
                  <button
                    onClick={() => setSelectedCandidate(null)}
                    className="rounded-full p-2 text-2xl text-gray-500 transition-colors hover:bg-gray-100 hover:text-gray-700"
                  >
                    ×
                  </button>
                </div>

                <div className="mb-8">
                  {/* Candidate image */}
                  <div className="mx-auto mb-6 h-40 w-40 overflow-hidden rounded-full border-4 border-blue-200">
                    <Image
                      src={selectedCandidate.image}
                      alt={selectedCandidate.name}
                      width={160}
                      height={160}
                      className="h-full w-full object-cover"
                    />
                  </div>
                </div>

                <div className="space-y-6">
                  <div className="rounded-lg bg-blue-50 p-4">
                    <h3 className="mb-2 text-lg font-bold text-gray-800">
                      🏛️ Political Party
                    </h3>
                    <p className="text-lg font-semibold text-blue-700">
                      {selectedCandidate.party}
                    </p>
                  </div>

                  <div className="rounded-lg bg-gray-50 p-4">
                    <h3 className="mb-3 text-lg font-bold text-gray-800">
                      📝 Biography & Background
                    </h3>
                    <p className="leading-relaxed text-gray-700">
                      {selectedCandidate.description}
                    </p>
                  </div>
                </div>

                <div className="mt-8 flex justify-between border-t pt-6">
                  <div className="flex gap-3">
                    <button
                      onClick={() => startEdit(selectedCandidate)}
                      className="rounded-lg bg-orange-500 px-6 py-3 font-semibold text-white transition-all duration-200 hover:bg-orange-600 hover:shadow-md"
                    >
                      ✏️ Edit
                    </button>
                    <button
                      onClick={() => setDeleteConfirmation(selectedCandidate)}
                      className="rounded-lg bg-red-500 px-6 py-3 font-semibold text-white transition-all duration-200 hover:bg-red-600 hover:shadow-md"
                    >
                      🗑️ Delete
                    </button>
                  </div>
                  <button
                    onClick={() => setSelectedCandidate(null)}
                    className="rounded-lg bg-gray-500 px-8 py-3 font-semibold text-white transition-all duration-200 hover:bg-gray-600 hover:shadow-md"
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
