import React from "react";
import { render, screen, act, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { CameraProvider, useCameraContext, Exercise } from "./CameraContext";

function TestComponent() {
  const {
    exercises,
    addExercise,
    updateExercise,
    deleteExercise,
    getExerciseById,
  } = useCameraContext();

  return (
    <div>
      <h1>Exercises: {exercises.length}</h1>
      <button
        onClick={() =>
          addExercise({
            name: "Test Exercise",
            videoUrl: "test-url.mp4",
            form: "good",
            date: new Date().toISOString(),
            duration: 60,
          })
        }
      >
        Add Exercise
      </button>

      <button
        onClick={() => {
          if (exercises.length > 0) {
            updateExercise(exercises[0].id, { name: "Updated Exercise" });
          }
        }}
      >
        Update Exercise
      </button>

      <button
        onClick={() => {
          if (exercises.length > 0) {
            deleteExercise(exercises[0].id);
          }
        }}
      >
        Delete Exercise
      </button>

      <button
        onClick={() => {
          if (exercises.length > 0) {
            const exercise = getExerciseById(exercises[0].id);
            if (exercise) {
              document.querySelector(
                '[data-testid="exercise-details"]',
              )!.textContent = `${exercise.name}-${exercise.form}`;
            }
          }
        }}
      >
        Get Exercise
      </button>

      <div data-testid="exercise-details"></div>

      <ul>
        {exercises.map((ex) => (
          <li key={ex.id} data-testid={`exercise-${ex.id}`}>
            {ex.name}
          </li>
        ))}
      </ul>
    </div>
  );
}

describe("CameraContext CRUD Operations", () => {
  beforeEach(() => {
    jest.clearAllMocks();

    window.localStorage.getItem = jest.fn().mockReturnValue(null);
  });

  test("should add a new exercise properly", async () => {
    render(
      <CameraProvider>
        <TestComponent />
      </CameraProvider>,
    );

    expect(screen.getByText("Exercises: 0")).toBeInTheDocument();

    await userEvent.click(screen.getByText("Add Exercise"));

    await waitFor(() => {
      expect(screen.getByText("Exercises: 1")).toBeInTheDocument();
    });

    expect(window.localStorage.setItem).toHaveBeenCalled();
    expect(screen.getByText("Test Exercise")).toBeInTheDocument();
  });

  test("should get existing exercise details", async () => {
    render(
      <CameraProvider>
        <TestComponent />
      </CameraProvider>,
    );

    await userEvent.click(screen.getByText("Add Exercise"));

    await waitFor(() => {
      expect(screen.getByText("Exercises: 1")).toBeInTheDocument();
    });

    await userEvent.click(screen.getByText("Get Exercise"));

    await waitFor(() => {
      const detailsElement = screen.getByTestId("exercise-details");
      expect(detailsElement.textContent).toContain("Test Exercise-good");
    });
  });

  test("should update an existing exercise", async () => {
    render(
      <CameraProvider>
        <TestComponent />
      </CameraProvider>,
    );

    await userEvent.click(screen.getByText("Add Exercise"));

    await waitFor(() => {
      expect(screen.getByText("Exercises: 1")).toBeInTheDocument();
    });

    await userEvent.click(screen.getByText("Update Exercise"));

    await waitFor(() => {
      expect(screen.queryByText("Test Exercise")).not.toBeInTheDocument();
      expect(screen.getByText("Updated Exercise")).toBeInTheDocument();
    });
  });

  test("should delete an existing exercise", async () => {
    render(
      <CameraProvider>
        <TestComponent />
      </CameraProvider>,
    );

    await userEvent.click(screen.getByText("Add Exercise"));

    await waitFor(() => {
      expect(screen.getByText("Exercises: 1")).toBeInTheDocument();
    });

    await userEvent.click(screen.getByText("Delete Exercise"));

    await waitFor(() => {
      expect(screen.getByText("Exercises: 0")).toBeInTheDocument();
    });
  });

  test("should load exercises from localStorage", async () => {
    const mockExercises = [
      {
        id: "123",
        name: "Stored Exercise",
        videoUrl: "stored-url.mp4",
        form: "medium",
        date: new Date().toISOString(),
        duration: 45,
      },
    ];

    window.localStorage.getItem = jest
      .fn()
      .mockReturnValue(JSON.stringify(mockExercises));

    render(
      <CameraProvider>
        <TestComponent />
      </CameraProvider>,
    );

    await waitFor(() => {
      expect(screen.getByText("Exercises: 1")).toBeInTheDocument();
      expect(screen.getByText("Stored Exercise")).toBeInTheDocument();
    });
  });

  test("should handle localStorage parsing errors", async () => {
    const originalConsoleError = console.error;
    console.error = jest.fn();

    window.localStorage.getItem = jest.fn().mockReturnValue("invalid-json");

    render(
      <CameraProvider>
        <TestComponent />
      </CameraProvider>,
    );

    expect(console.error).toHaveBeenCalled();

    expect(screen.getByText("Exercises: 0")).toBeInTheDocument();

    console.error = originalConsoleError;
  });
});
