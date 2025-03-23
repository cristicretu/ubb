import React from "react";
import { render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";
import ExerciseForm from "./ExerciseForm";
import { CameraProvider } from "./CameraContext";

jest.mock("sonner", () => ({
  toast: {
    success: jest.fn(),
    error: jest.fn(),
  },
}));

describe("ExerciseForm Integration Tests", () => {
  const mockVideoUrl = "test-video-url.mp4";
  const mockDuration = 120;
  const mockOnCancel = jest.fn();
  const mockOnSave = jest.fn();

  beforeEach(() => {
    jest.clearAllMocks();
  });

  test("should create a new exercise when form is submitted", async () => {
    render(
      <CameraProvider>
        <ExerciseForm
          videoUrl={mockVideoUrl}
          duration={mockDuration}
          onCancel={mockOnCancel}
          onSave={mockOnSave}
        />
      </CameraProvider>,
    );

    const nameInput = screen.getByPlaceholderText("Enter exercise name");
    await userEvent.type(nameInput, "Push Ups");

    await userEvent.click(screen.getByText("Good"));

    await userEvent.click(screen.getByText("Save Exercise"));

    await waitFor(() => {
      expect(mockOnSave).toHaveBeenCalled();
    });

    const { toast } = require("sonner");
    expect(toast.success).toHaveBeenCalledWith(
      'Exercise "Push Ups" saved successfully',
    );
  });

  test("should update an existing exercise when form is submitted", async () => {
    const initialData = {
      id: "123",
      name: "Old Exercise",
      form: "bad" as const,
      videoUrl: "old-url.mp4",
      date: new Date().toISOString(),
      duration: 60,
    };

    render(
      <CameraProvider>
        <ExerciseForm
          videoUrl={mockVideoUrl}
          duration={mockDuration}
          onCancel={mockOnCancel}
          onSave={mockOnSave}
          initialData={initialData}
        />
      </CameraProvider>,
    );

    const nameInput = screen.getByDisplayValue("Old Exercise");
    await userEvent.clear(nameInput);
    await userEvent.type(nameInput, "Updated Exercise");

    await userEvent.click(screen.getByText("Medium"));

    await userEvent.click(screen.getByText("Update Exercise"));

    await waitFor(() => {
      expect(mockOnSave).toHaveBeenCalled();
    });

    const { toast } = require("sonner");
    expect(toast.success).toHaveBeenCalledWith(
      'Exercise "Updated Exercise" updated successfully',
    );
  });

  test("should display validation error for empty exercise name", async () => {
    render(
      <CameraProvider>
        <ExerciseForm
          videoUrl={mockVideoUrl}
          duration={mockDuration}
          onCancel={mockOnCancel}
          onSave={mockOnSave}
        />
      </CameraProvider>,
    );

    const saveButton = screen.getByText("Save Exercise");

    expect(saveButton).toBeDisabled();

    const nameInput = screen.getByPlaceholderText("Enter exercise name");
    await userEvent.type(nameInput, "Test");
    await userEvent.clear(nameInput);
    await userEvent.tab();

    expect(
      screen.getByText("Exercise name cannot be empty"),
    ).toBeInTheDocument();
  });

  test("should display validation error for name with numbers", async () => {
    render(
      <CameraProvider>
        <ExerciseForm
          videoUrl={mockVideoUrl}
          duration={mockDuration}
          onCancel={mockOnCancel}
          onSave={mockOnSave}
        />
      </CameraProvider>,
    );

    const nameInput = screen.getByPlaceholderText("Enter exercise name");
    await userEvent.type(nameInput, "Push Ups 123");
    await userEvent.tab();

    expect(
      screen.getByText("Exercise name cannot contain numbers"),
    ).toBeInTheDocument();

    const saveButton = screen.getByText("Save Exercise");
    expect(saveButton).not.toBeDisabled();

    await userEvent.click(saveButton);

    expect(mockOnSave).not.toHaveBeenCalled();
  });

  test("should call onCancel when cancel button is clicked", async () => {
    render(
      <CameraProvider>
        <ExerciseForm
          videoUrl={mockVideoUrl}
          duration={mockDuration}
          onCancel={mockOnCancel}
          onSave={mockOnSave}
        />
      </CameraProvider>,
    );

    await userEvent.click(screen.getByText("Cancel"));

    expect(mockOnCancel).toHaveBeenCalled();
    expect(mockOnSave).not.toHaveBeenCalled();
  });

  test("should submit when Enter key is pressed in name input", async () => {
    render(
      <CameraProvider>
        <ExerciseForm
          videoUrl={mockVideoUrl}
          duration={mockDuration}
          onCancel={mockOnCancel}
          onSave={mockOnSave}
        />
      </CameraProvider>,
    );

    const nameInput = screen.getByPlaceholderText("Enter exercise name");
    await userEvent.type(nameInput, "Squats{enter}");

    await waitFor(() => {
      expect(mockOnSave).toHaveBeenCalled();
    });
  });
});
