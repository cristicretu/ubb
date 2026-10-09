import { NextRequest, NextResponse } from "next/server";
import { candidates, broadcastUpdate } from "../route";

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

// Global interval storage
let generationInterval: NodeJS.Timeout | null = null;
let isGenerating = false;

// Generate random candidate
function generateRandomCandidate() {
  const firstName = firstNames[Math.floor(Math.random() * firstNames.length)]!;
  const lastName = lastNames[Math.floor(Math.random() * lastNames.length)]!;
  const party = parties[Math.floor(Math.random() * parties.length)]!;
  const description =
    descriptions[Math.floor(Math.random() * descriptions.length)]!;

  const initials = firstName.charAt(0) + lastName.charAt(0);

  return {
    id: Math.max(...candidates.map((c) => c.id), 0) + 1,
    name: `${firstName} ${lastName}`,
    image: `https://placehold.co/600x400/000000/FFFFFF.png?text=${initials}`,
    party,
    description,
  };
}

// POST - Start generation
export async function POST() {
  if (isGenerating) {
    return NextResponse.json({
      success: false,
      error: "Generation is already running",
    });
  }

  isGenerating = true;

  // Start generating candidates every 2 seconds
  generationInterval = setInterval(() => {
    const newCandidate = generateRandomCandidate();
    candidates.push(newCandidate);

    // Broadcast update to all connected clients
    broadcastUpdate("candidate_generated", {
      candidate: newCandidate,
      candidates: candidates,
    });
  }, 2000);

  // Broadcast generation started
  broadcastUpdate("generation_started", {
    message: "Random candidate generation started",
    isGenerating: true,
  });

  return NextResponse.json({
    success: true,
    message: "Random candidate generation started",
    isGenerating: true,
  });
}

// DELETE - Stop generation
export async function DELETE() {
  if (!isGenerating) {
    return NextResponse.json({
      success: false,
      error: "Generation is not running",
    });
  }

  if (generationInterval) {
    clearInterval(generationInterval);
    generationInterval = null;
  }

  isGenerating = false;

  // Broadcast generation stopped
  broadcastUpdate("generation_stopped", {
    message: "Random candidate generation stopped",
    isGenerating: false,
  });

  return NextResponse.json({
    success: true,
    message: "Random candidate generation stopped",
    isGenerating: false,
  });
}

// GET - Check generation status
export async function GET() {
  return NextResponse.json({
    success: true,
    isGenerating,
  });
}
