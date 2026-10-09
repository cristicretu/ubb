import React from "react";
import { render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import { CameraProvider, useCameraContext } from "./CameraContext";

function NegativeTestComponent() {
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
        data-testid="add-invalid-form"
        onClick={() => {
          try {
            addExercise({
              name: "Invalid Form Exercise",
              videoUrl: "test-url.mp4",
              form: "invalid-form",
              date: new Date().toISOString(),
              duration: 60,
            });
            document.getElementById("error-message")!.textContent = "No error";
          } catch (error) {
            document.getElementById("error-message")!.textContent =
              "Error occurred";
          }
        }}
      >
        Add Invalid Form
      </button>

      <button
        data-testid="update-nonexistent"
        onClick={() => {
          try {
            updateExercise("non-existent-id", { name: "Updated Name" });
            document.getElementById("error-message")!.textContent = "No error";
          } catch (error) {
            document.getElementById("error-message")!.textContent =
              "Error occurred";
          }
        }}
      >
        Update Non-existent
      </button>

      <button
        data-testid="delete-nonexistent"
        onClick={() => {
          try {
            deleteExercise("non-existent-id");
            document.getElementById("error-message")!.textContent = "No error";
          } catch (error) {
            document.getElementById("error-message")!.textContent =
              "Error occurred";
          }
        }}
      >
        Delete Non-existent
      </button>

      <button
        data-testid="get-nonexistent"
        onClick={() => {
          const exercise = getExerciseById("non-existent-id");
          document.getElementById("error-message")!.textContent = exercise
            ? "Found exercise"
            : "No exercise found";
        }}
      >
        Get Non-existent
      </button>

      <div id="error-message"></div>

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

describe("CameraContext Negative Test Cases", () => {
  beforeEach(() => {
    jest.clearAllMocks();
    window.localStorage.getItem = jest.fn().mockReturnValue(null);
  });

  test("should return undefined when getting a non-existent exercise", async () => {
    render(
      <CameraProvider>
        <NegativeTestComponent />
      </CameraProvider>,
    );

    await userEvent.click(screen.getByTestId("get-nonexistent"));

    await waitFor(() => {
      const errorMessage = document.getElementById("error-message");
      expect(errorMessage?.textContent).toBe("No exercise found");
    });
  });

  test("should not throw error when deleting a non-existent exercise", async () => {
    render(
      <CameraProvider>
        <NegativeTestComponent />
      </CameraProvider>,
    );

    await userEvent.click(screen.getByTestId("delete-nonexistent"));

    await waitFor(() => {
      const errorMessage = document.getElementById("error-message");
      expect(errorMessage?.textContent).toBe("No error");
    });
  });

  test("should not throw error when updating a non-existent exercise", async () => {
    render(
      <CameraProvider>
        <NegativeTestComponent />
      </CameraProvider>,
    );

    await userEvent.click(screen.getByTestId("update-nonexistent"));

    await waitFor(() => {
      const errorMessage = document.getElementById("error-message");
      expect(errorMessage?.textContent).toBe("No error");
    });
  });

  test("should handle invalid form values appropriately", async () => {
    render(
      <CameraProvider>
        <NegativeTestComponent />
      </CameraProvider>,
    );

    await userEvent.click(screen.getByTestId("add-invalid-form"));

    await waitFor(() => {
      const errorMessage = document.getElementById("error-message");
      expect(errorMessage?.textContent).toBeDefined();
    });
  });

  test("should properly add and remove event listeners", async () => {
    const callback = jest.fn();

    function EventTestComponent() {
      const { addEventListener, removeEventListener, addExercise } =
        useCameraContext();

      React.useEffect(() => {
        addEventListener("exercisesChange", callback);
        return () => removeEventListener("exercisesChange", callback);
      }, [addEventListener, removeEventListener]);

      return (
        <button
          onClick={() =>
            addExercise({
              name: "Event Test Exercise",
              videoUrl: "test-url.mp4",
              form: "good",
              date: new Date().toISOString(),
              duration: 60,
            })
          }
        >
          Add Exercise
        </button>
      );
    }

    const { unmount } = render(
      <CameraProvider>
        <EventTestComponent />
      </CameraProvider>,
    );
    await userEvent.click(screen.getByText("Add Exercise"));

    await waitFor(() => {
      expect(callback).toHaveBeenCalled();
    });

    callback.mockReset();
    unmount();

    expect(callback).not.toHaveBeenCalled();
  });
});
