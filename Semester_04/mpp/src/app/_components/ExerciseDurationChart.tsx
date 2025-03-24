"use client";

import { useEffect, useState } from "react";
import { Exercise } from "./CameraContext";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Legend,
} from "recharts";

interface ExerciseDurationChartProps {
  exercises: Exercise[];
}

export default function ExerciseDurationChart({
  exercises,
}: ExerciseDurationChartProps) {
  const [chartData, setChartData] = useState<
    Array<{
      date: string;
      displayDate: string;
      duration: number;
      durationMinutes: number;
    }>
  >([]);

  useEffect(() => {
    const prepareData = () => {
      if (!exercises.length) return [];

      const sortedExercises = [...exercises].sort(
        (a, b) => new Date(a.date).getTime() - new Date(b.date).getTime(),
      );

      // Process data for the chart
      return sortedExercises.map((exercise) => {
        const date = new Date(exercise.date);
        const displayDate = date.toLocaleDateString("en-US", {
          month: "short",
          day: "numeric",
        });

        return {
          date: date.toISOString().split("T")[0],
          displayDate,
          duration: exercise.duration,
          durationMinutes: Math.round((exercise.duration / 60) * 10) / 10,
        };
      });
    };

    setChartData(prepareData());
  }, [exercises]);

  if (!exercises.length) {
    return null;
  }

  // Function to format the tooltip
  const CustomTooltip = ({ active, payload, label }: any) => {
    if (active && payload && payload.length) {
      const data = payload[0].payload;
      return (
        <div className="rounded border border-gray-200 bg-white p-2 shadow-sm">
          <p className="font-medium">{data.displayDate}</p>
          <p className="text-gray-600">
            {`Duration: ${Math.floor(data.duration / 60)}:${String(
              data.duration % 60,
            ).padStart(2, "0")}`}
          </p>
        </div>
      );
    }
    return null;
  };

  const formatYAxis = (value: number) => {
    return `${value} min`;
  };

  return (
    <Card className="border-border border shadow-sm">
      <CardHeader className="pb-2">
        <CardTitle className="text-lg font-medium">Exercise Duration</CardTitle>
        <CardDescription>Duration of each exercise over time</CardDescription>
      </CardHeader>
      <CardContent>
        <div className="h-[250px] w-full">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart
              data={chartData}
              margin={{ top: 5, right: 30, left: 20, bottom: 5 }}
            >
              <CartesianGrid strokeDasharray="3 3" stroke="#e5e7eb" />
              <XAxis
                dataKey="displayDate"
                tickLine={false}
                axisLine={{ stroke: "#e5e7eb" }}
                tick={{ fontSize: 12 }}
              />
              <YAxis
                tickFormatter={formatYAxis}
                tickLine={false}
                axisLine={{ stroke: "#e5e7eb" }}
                tick={{ fontSize: 12 }}
              />
              <Tooltip content={<CustomTooltip />} />
              <Line
                type="monotone"
                dataKey="durationMinutes"
                name="Duration"
                stroke="#3b82f6"
                strokeWidth={2}
                dot={{ r: 4, fill: "#3b82f6", strokeWidth: 2, stroke: "white" }}
                activeDot={{ r: 6, fill: "#1d4ed8" }}
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      </CardContent>
    </Card>
  );
}
