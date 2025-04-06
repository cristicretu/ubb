import React, { useState } from "react";
import { Exercise } from "../models/Exercise";
import { deleteExercise } from "../services/ExerciseService";
import { toast } from "sonner";

const ExerciseList: React.FC = () => {
  const [exercises, setExercises] = useState<Exercise[]>([]);
  const [selectedExercise, setSelectedExercise] = useState<Exercise | null>(
    null,
  );
  const [showEditForm, setShowEditForm] = useState(false);
  const [page, setPage] = useState(1);

  const handleEdit = (exercise: Exercise) => {
    console.log("Opening edit form for exercise:", exercise.id, exercise);
    setSelectedExercise(exercise);
    setShowEditForm(true);
  };

  const handleDelete = async (id: string) => {
    console.log("Attempting to delete exercise with ID:", id);
    try {
      await deleteExercise(id);
      setPage(1);
      toast.success("Exercise deleted successfully");
    } catch (error) {
      console.error("Error deleting exercise:", error);
      toast.error("Failed to delete exercise");
    }
  };

  return <div>{/* Render your exercise list components here */}</div>;
};

export default ExerciseList;
