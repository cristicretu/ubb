"use client";

import { useState, useEffect } from "react";
import { Exercise, useCameraContext } from "./CameraContext";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import {
  AreaChart,
  Area,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
} from "recharts";

interface ExerciseProgressChartProps {
  exercises: Exercise[];
}

interface ChartDataItem {
  date: string;
  displayDate: string;
  intensity: number;
  count: number;
  form: {
    good: number;
    medium: number;
    bad: number;
  };
}

export default function ExerciseProgressChart({
  exercises,
}: ExerciseProgressChartProps) {
  const [averageIntensityByDay, setAverageIntensityByDay] = useState<
    ChartDataItem[]
  >([]);
  const { addEventListener, removeEventListener } = useCameraContext();

  const calculateIntensity = () => {
    if (!exercises.length) return [];

    const exercisesByDay: {
      [key: string]: {
        exercises: Exercise[];
        totalDuration: number;
      };
    } = {};

    exercises.forEach((exercise) => {
      const date = new Date(exercise.date);
      const dateKey = date.toISOString().split("T")[0];

      if (!exercisesByDay[dateKey as keyof typeof exercisesByDay]) {
        exercisesByDay[dateKey as keyof typeof exercisesByDay] = {
          exercises: [],
          totalDuration: 0,
        };
      }

      exercisesByDay[dateKey as keyof typeof exercisesByDay].exercises.push(
        exercise,
      );
      exercisesByDay[dateKey as keyof typeof exercisesByDay].totalDuration +=
        exercise.duration;
    });

    const intensityData = Object.entries(exercisesByDay).map(
      ([dateKey, data]) => {
        const { exercises, totalDuration } = data;
        const count = exercises.length;

        const formCounts = {
          good: exercises.filter((ex) => ex.form === "good").length,
          medium: exercises.filter((ex) => ex.form === "medium").length,
          bad: exercises.filter((ex) => ex.form === "bad").length,
        };

        const qualityFactor =
          (formCounts.good * 1.0 +
            formCounts.medium * 0.7 +
            formCounts.bad * 0.4) /
          count;

        const intensity = (totalDuration * qualityFactor) / count;

        const date = new Date(dateKey);
        const displayDate = date.toLocaleDateString("en-US", {
          month: "short",
          day: "numeric",
        });

        return {
          date: dateKey,
          displayDate,
          intensity,
          count,
          form: formCounts,
        };
      },
    );

    return intensityData.sort((a, b) => a.date.localeCompare(b.date));
  };

  const updateChartData = () => {
    const intensityData = calculateIntensity();
    setAverageIntensityByDay(intensityData);
  };

  useEffect(() => {
    // Update chart data when exercises prop changes
    updateChartData();

    // Subscribe to exercise changes to update the chart when data changes
    addEventListener("exercisesChange", updateChartData);

    // Cleanup
    return () => {
      removeEventListener("exercisesChange", updateChartData);
    };
  }, [exercises, addEventListener, removeEventListener]);

  // Always render the chart component, even if there's no data
  return (
    <Card className="border-border border shadow-sm">
      <CardHeader className="pb-2">
        <CardTitle className="text-lg font-medium">
          Exercise Intensity
        </CardTitle>
        <CardDescription>
          Workout intensity over time based on duration and form quality
        </CardDescription>
      </CardHeader>
      <CardContent>
        <div className="h-[250px] w-full">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart
              data={averageIntensityByDay}
              margin={{ top: 10, right: 30, left: 0, bottom: 0 }}
            >
              <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
              <XAxis
                dataKey="displayDate"
                tickLine={false}
                axisLine={{ stroke: "#e5e7eb" }}
              />
              <YAxis
                label={{
                  value: "Intensity",
                  angle: -90,
                  position: "insideLeft",
                  style: {
                    textAnchor: "middle",
                    fontSize: 12,
                    fill: "#6b7280",
                  },
                }}
                tickLine={false}
                axisLine={{ stroke: "#e5e7eb" }}
              />
              <Tooltip content={CustomTooltip} />
              <Area
                type="monotone"
                dataKey="intensity"
                stroke="#3b82f6"
                fillOpacity={0.3}
                fill="#93c5fd"
                strokeWidth={2}
                activeDot={{
                  r: 6,
                  strokeWidth: 2,
                  stroke: "white",
                  fill: "#2563eb",
                }}
              />
            </AreaChart>
          </ResponsiveContainer>
        </div>
      </CardContent>
    </Card>
  );
}

// Custom tooltip component
const CustomTooltip = ({ active, payload, label }: any) => {
  if (active && payload && payload.length) {
    const data = payload[0].payload as ChartDataItem;
    return (
      <div className="rounded border border-gray-200 bg-white p-2 shadow-sm">
        <p className="font-medium">{data.displayDate}</p>
        <p className="text-gray-600">Intensity: {data.intensity.toFixed(1)}</p>
        <p className="text-gray-600">Exercises: {data.count}</p>
        <div className="mt-1 text-xs">
          <p className="text-green-600">Good: {data.form.good}</p>
          <p className="text-yellow-500">Medium: {data.form.medium}</p>
          <p className="text-red-500">Bad: {data.form.bad}</p>
        </div>
      </div>
    );
  }
  return null;
};
