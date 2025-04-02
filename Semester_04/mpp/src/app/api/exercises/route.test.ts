import { NextResponse } from "next/server";
import { GET, POST } from "./route";
import { exerciseStore } from "../_lib/store";

jest.mock("../_lib/store", () => {
  const mockExercises = [
    {
      id: "1",
      name: "Squat",
      videoUrl: "https://example.com/squat.mp4",
      form: "good",
      date: "2023-01-01T10:00:00.000Z",
      duration: 30,
    },
    {
      id: "2",
      name: "Push-up",
      videoUrl: "https://example.com/pushup.mp4",
      form: "medium",
      date: "2023-01-02T10:00:00.000Z",
      duration: 45,
    },
    {
      id: "3",
      name: "Deadlift",
      videoUrl: "https://example.com/deadlift.mp4",
      form: "bad",
      date: "2023-01-03T10:00:00.000Z",
      duration: 60,
    },
  ];

  return {
    exerciseStore: {
      getAll: jest.fn(() => [...mockExercises]),
      add: jest.fn((exercise) => ({
        ...exercise,
        id: "new-id",
      })),
    },
  };
});

jest.mock("next/server", () => {
  return {
    NextResponse: {
      json: jest.fn((data, options) => ({
        status: options?.status || 200,
        json: async () => data,
      })),
    },
  };
});

describe("GET /api/exercises", () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it("should return all exercises without filters", async () => {
    const request = {
      url: "http://localhost:3000/api/exercises",
      nextUrl: {
        searchParams: new URLSearchParams(""),
      },
    };

    const response = await GET(request);
    const data = await response.json();

    expect(response.status).toBe(200);
    expect(data.exercises).toHaveLength(3);
    expect(data.total).toBe(3);
    expect(data.page).toBe(1);
    expect(data.limit).toBe(10);
    expect(data.totalPages).toBe(1);
  });

  it("should filter exercises by form", async () => {
    const request = {
      url: "http://localhost:3000/api/exercises?form=good",
      nextUrl: {
        searchParams: new URLSearchParams("form=good"),
      },
    };

    const response = await GET(request);
    const data = await response.json();

    expect(exerciseStore.getAll).toHaveBeenCalled();
    expect(data.exercises.length).toBe(1);
    expect(data.exercises[0].form).toBe("good");
  });

  it("should filter exercises by search term", async () => {
    const request = {
      url: "http://localhost:3000/api/exercises?search=squat",
      nextUrl: {
        searchParams: new URLSearchParams("search=squat"),
      },
    };

    const response = await GET(request);
    const data = await response.json();

    expect(exerciseStore.getAll).toHaveBeenCalled();
    expect(data.exercises.length).toBe(1);
    expect(data.exercises[0].name).toBe("Squat");
  });

  it("should sort exercises by date-newest", async () => {
    const request = {
      url: "http://localhost:3000/api/exercises?sortBy=date-newest",
      nextUrl: {
        searchParams: new URLSearchParams("sortBy=date-newest"),
      },
    };

    const response = await GET(request);
    const data = await response.json();

    expect(exerciseStore.getAll).toHaveBeenCalled();
    expect(data.exercises[0].id).toBe("3");
  });

  it("should sort exercises by duration-highest", async () => {
    const request = {
      url: "http://localhost:3000/api/exercises?sortBy=duration-highest",
      nextUrl: {
        searchParams: new URLSearchParams("sortBy=duration-highest"),
      },
    };

    const response = await GET(request);
    const data = await response.json();

    expect(exerciseStore.getAll).toHaveBeenCalled();
    expect(data.exercises[0].id).toBe("3");
  });

  it("should paginate results", async () => {
    const request = {
      url: "http://localhost:3000/api/exercises?page=2&limit=1",
      nextUrl: {
        searchParams: new URLSearchParams("page=2&limit=1"),
      },
    };

    const response = await GET(request);
    const data = await response.json();

    expect(exerciseStore.getAll).toHaveBeenCalled();
    expect(data.exercises.length).toBe(1);
    expect(data.page).toBe(2);
    expect(data.limit).toBe(1);
    expect(data.total).toBe(3);
    expect(data.totalPages).toBe(3);
  });
});

describe("POST /api/exercises", () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it("should create a new exercise with valid data", async () => {
    const validExercise = {
      name: "New Exercise",
      videoUrl: "https://example.com/new.mp4",
      form: "good",
      date: "2023-01-04T10:00:00.000Z",
      duration: 25,
    };

    const request = {
      url: "http://localhost:3000/api/exercises",
      method: "POST",
      json: jest.fn().mockResolvedValue(validExercise),
    };

    const response = await POST(request);
    const data = await response.json();

    expect(response.status).toBe(201);
    expect(exerciseStore.add).toHaveBeenCalledWith(validExercise);
    expect(data).toHaveProperty("id", "new-id");
  });

  it("should return validation error for invalid data", async () => {
    const invalidExercise = {
      name: "",
      videoUrl: "not-a-url",
      form: "invalid-form",
      date: "not-a-date",
      duration: -5,
    };

    const request = {
      url: "http://localhost:3000/api/exercises",
      method: "POST",
      json: jest.fn().mockResolvedValue(invalidExercise),
    };

    const response = await POST(request);
    const data = await response.json();

    expect(response.status).toBe(400);
    expect(data).toHaveProperty("error", "Validation failed");
    expect(data).toHaveProperty("details");
    expect(exerciseStore.add).not.toHaveBeenCalled();
  });

  it("should handle JSON parsing error", async () => {
    const request = {
      url: "http://localhost:3000/api/exercises",
      method: "POST",
      json: jest.fn().mockRejectedValue(new Error("Invalid JSON")),
    };

    const response = await POST(request);
    const data = await response.json();

    expect(response.status).toBe(500);
    expect(data).toHaveProperty("error", "Failed to create exercise");
  });
});
