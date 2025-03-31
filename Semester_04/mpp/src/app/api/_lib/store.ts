export interface Exercise {
  id: string;
  name: string;
  videoUrl: string;
  form: "bad" | "medium" | "good";
  date: string;
  duration: number;
}

class ExerciseStore {
  private exercises: Exercise[] = [];

  getAll(): Exercise[] {
    return this.exercises;
  }

  getById(id: string): Exercise | undefined {
    return this.exercises.find((ex) => ex.id === id);
  }

  add(exercise: Omit<Exercise, "id">): Exercise {
    const newExercise: Exercise = {
      ...exercise,
      id: Date.now().toString(),
    };

    this.exercises = [newExercise, ...this.exercises];
    return newExercise;
  }

  update(id: string, updates: Partial<Omit<Exercise, "id">>): Exercise | null {
    const index = this.exercises.findIndex((ex) => ex.id === id);

    if (index === -1) {
      console.error(`Exercise with ID ${id} not found for update operation`);
      return null;
    }

    const updatedExercise = {
      ...this.exercises[index],
      ...updates,
    };

    this.exercises[index] = updatedExercise;
    return updatedExercise;
  }

  delete(id: string): Exercise | null {
    const index = this.exercises.findIndex((ex) => ex.id === id);

    if (index === -1) {
      return null;
    }

    const [deletedExercise] = this.exercises.splice(index, 1);
    return deletedExercise;
  }

  filter(criteria: Partial<Exercise>): Exercise[] {
    return this.exercises.filter((exercise) => {
      return Object.entries(criteria).every(([key, value]) => {
        return exercise[key as keyof Exercise] === value;
      });
    });
  }
}

export const exerciseStore = new ExerciseStore();
