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

interface CandidateFormData {
  name: string;
  image: string;
  party: string;
  description: string;
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

    setCandidates((prev) => [...prev, newCandidate]);
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

    setCandidates((prev) =>
      prev.map((candidate) =>
        candidate.id === editingCandidate.id
          ? { ...candidate, ...formData }
          : candidate,
      ),
    );

    resetForm();
    showMessage("success", `${formData.name} has been updated successfully!`);
  };

  const handleDelete = (candidate: Candidate) => {
    setCandidates((prev) => prev.filter((c) => c.id !== candidate.id));
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
    <main className="min-h-screen bg-gradient-to-br from-blue-50 to-red-50 p-4">
      <div className="container mx-auto max-w-6xl">
        {/* Header */}
        <div className="mb-8 text-center">
          <h1 className="mb-4 text-4xl font-bold text-gray-800">
            🇷🇴 Romanian Political Candidates
          </h1>
          <p className="text-lg text-gray-600">
            Explore and manage profiles of prominent Romanian political figures
          </p>
        </div>

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
