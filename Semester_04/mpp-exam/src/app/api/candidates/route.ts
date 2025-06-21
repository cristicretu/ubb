import { NextRequest, NextResponse } from "next/server";

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

// In-memory storage for candidates
let candidates: Candidate[] = [
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

const wsConnections = new Set<any>();

export function addWebSocketConnection(ws: any) {
  wsConnections.add(ws);
}

export function removeWebSocketConnection(ws: any) {
  wsConnections.delete(ws);
}

function broadcastUpdate(type: string, data: any) {
  const message = JSON.stringify({ type, data });

  wsConnections.forEach((ws) => {
    try {
      if (ws.readyState === 1) {
        ws.send(message);
      } else {
        wsConnections.delete(ws);
      }
    } catch (error) {
      console.error("Error sending WebSocket message:", error);
      wsConnections.delete(ws);
    }
  });
}

// GET - Fetch all candidates
export async function GET() {
  return NextResponse.json({
    success: true,
    data: candidates,
  });
}

// POST - Create new candidate
export async function POST(request: NextRequest) {
  try {
    const body: CandidateFormData = await request.json();

    // Validation
    if (!body.name?.trim()) {
      return NextResponse.json(
        { success: false, error: "Name is required" },
        { status: 400 },
      );
    }

    if (!body.party?.trim()) {
      return NextResponse.json(
        { success: false, error: "Party is required" },
        { status: 400 },
      );
    }

    if (!body.description?.trim()) {
      return NextResponse.json(
        { success: false, error: "Description is required" },
        { status: 400 },
      );
    }

    if (!body.image?.trim()) {
      return NextResponse.json(
        { success: false, error: "Image URL is required" },
        { status: 400 },
      );
    }

    // Check for duplicate names
    const existingCandidate = candidates.find(
      (c) => c.name.toLowerCase() === body.name.toLowerCase(),
    );

    if (existingCandidate) {
      return NextResponse.json(
        { success: false, error: "A candidate with this name already exists" },
        { status: 409 },
      );
    }

    // Create new candidate
    const newCandidate: Candidate = {
      id: Math.max(...candidates.map((c) => c.id), 0) + 1,
      name: body.name.trim(),
      image: body.image.trim(),
      party: body.party.trim(),
      description: body.description.trim(),
    };

    candidates.push(newCandidate);

    // Broadcast update to all connected clients
    broadcastUpdate("candidate_created", {
      candidate: newCandidate,
      candidates: candidates,
    });

    return NextResponse.json({
      success: true,
      data: newCandidate,
      message: `${newCandidate.name} has been added successfully!`,
    });
  } catch (error) {
    return NextResponse.json(
      { success: false, error: "Invalid request body" },
      { status: 400 },
    );
  }
}

// PUT - Update existing candidate
export async function PUT(request: NextRequest) {
  try {
    const body: Candidate = await request.json();

    if (!body.id) {
      return NextResponse.json(
        { success: false, error: "Candidate ID is required" },
        { status: 400 },
      );
    }

    // Find candidate
    const candidateIndex = candidates.findIndex((c) => c.id === body.id);
    if (candidateIndex === -1) {
      return NextResponse.json(
        { success: false, error: "Candidate not found" },
        { status: 404 },
      );
    }

    // Validation
    if (!body.name?.trim()) {
      return NextResponse.json(
        { success: false, error: "Name is required" },
        { status: 400 },
      );
    }

    // Check for duplicate names (excluding current candidate)
    const existingCandidate = candidates.find(
      (c) =>
        c.id !== body.id && c.name.toLowerCase() === body.name.toLowerCase(),
    );

    if (existingCandidate) {
      return NextResponse.json(
        { success: false, error: "A candidate with this name already exists" },
        { status: 409 },
      );
    }

    // Update candidate
    candidates[candidateIndex] = {
      ...candidates[candidateIndex],
      name: body.name.trim(),
      image: body.image.trim(),
      party: body.party.trim(),
      description: body.description.trim(),
    };

    // Broadcast update
    broadcastUpdate("candidate_updated", {
      candidate: candidates[candidateIndex],
      candidates: candidates,
    });

    return NextResponse.json({
      success: true,
      data: candidates[candidateIndex],
      message: `${candidates[candidateIndex].name} has been updated successfully!`,
    });
  } catch (error) {
    return NextResponse.json(
      { success: false, error: "Invalid request body" },
      { status: 400 },
    );
  }
}

// DELETE - Delete candidate
export async function DELETE(request: NextRequest) {
  try {
    const url = new URL(request.url);
    const id = parseInt(url.searchParams.get("id") || "");

    if (!id) {
      return NextResponse.json(
        { success: false, error: "Candidate ID is required" },
        { status: 400 },
      );
    }

    // Find candidate
    const candidateIndex = candidates.findIndex((c) => c.id === id);
    if (candidateIndex === -1) {
      return NextResponse.json(
        { success: false, error: "Candidate not found" },
        { status: 404 },
      );
    }

    const deletedCandidate = candidates[candidateIndex];
    candidates.splice(candidateIndex, 1);

    // Broadcast update
    broadcastUpdate("candidate_deleted", {
      candidate: deletedCandidate,
      candidates: candidates,
    });

    return NextResponse.json({
      success: true,
      data: deletedCandidate,
      message: `${deletedCandidate.name} has been deleted successfully!`,
    });
  } catch (error) {
    return NextResponse.json(
      { success: false, error: "Invalid request" },
      { status: 400 },
    );
  }
}

// Export candidates and connections for use in other files
export { candidates, wsConnections, broadcastUpdate };
