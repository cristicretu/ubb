"use client";

import { useState, useEffect } from "react";
import { Exercise } from "./CameraContext";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import {
  BarChart,
  Bar,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
  Cell,
} from "recharts";

interface WeeklyProgressChartProps {
  exercises: Exercise[];
}

interface DayData {
  name: string;
  value: number;
  bad: number;
  medium: number;
  good: number;
  total: number;
}

export default function WeeklyProgressChart({
  exercises,
}: WeeklyProgressChartProps) {
  const [weeklyData, setWeeklyData] = useState<DayData[]>([]);

  useEffect(() => {
    const calculateWeeklyData = () => {
      const days = [
        { name: "Sun", value: 0, bad: 0, medium: 0, good: 0, total: 0 },
        { name: "Mon", value: 0, bad: 0, medium: 0, good: 0, total: 0 },
        { name: "Tue", value: 0, bad: 0, medium: 0, good: 0, total: 0 },
        { name: "Wed", value: 0, bad: 0, medium: 0, good: 0, total: 0 },
        { name: "Thu", value: 0, bad: 0, medium: 0, good: 0, total: 0 },
        { name: "Fri", value: 0, bad: 0, medium: 0, good: 0, total: 0 },
        { name: "Sat", value: 0, bad: 0, medium: 0, good: 0, total: 0 },
      ];

      exercises.forEach((exercise) => {
        const date = new Date(exercise.date);
        const dayIndex = date.getDay(); // 0 = Sunday, 6 = Saturday
        days[dayIndex].value += 1;
        days[dayIndex].total += exercise.duration;

        // Increment form quality counter
        if (exercise.form === "bad") days[dayIndex].bad += 1;
        else if (exercise.form === "medium") days[dayIndex].medium += 1;
        else if (exercise.form === "good") days[dayIndex].good += 1;
      });

      return days;
    };

    setWeeklyData(calculateWeeklyData());
  }, [exercises]);

  if (!exercises.length) {
    return null;
  }

  // Custom tooltip for the chart
  const CustomTooltip = ({ active, payload, label }: any) => {
    if (active && payload && payload.length) {
      const data = payload[0].payload as DayData;
      const minutes = Math.floor(data.total / 60);
      const seconds = data.total % 60;
      const duration = `${minutes}:${seconds.toString().padStart(2, "0")}`;

      return (
        <div className="rounded border border-gray-200 bg-white p-2 shadow-sm">
          <p className="font-medium">{data.name}</p>
          <p className="text-gray-600">Total exercises: {data.value}</p>
          <p className="text-gray-600">Total duration: {duration}</p>
          <div className="mt-1 text-xs">
            <p className="text-green-600">Good: {data.good}</p>
            <p className="text-yellow-500">Medium: {data.medium}</p>
            <p className="text-red-500">Bad: {data.bad}</p>
          </div>
        </div>
      );
    }
    return null;
  };

  return (
    <Card className="border-border border shadow-sm">
      <CardHeader className="pb-2">
        <CardTitle className="text-lg font-medium">Weekly Pattern</CardTitle>
        <CardDescription>
          Number of exercises by day of the week
        </CardDescription>
      </CardHeader>
      <CardContent>
        <div className="h-[250px] w-full">
          <ResponsiveContainer width="100%" height="100%">
            <BarChart
              data={weeklyData}
              margin={{ top: 10, right: 30, left: 0, bottom: 5 }}
            >
              <CartesianGrid
                strokeDasharray="3 3"
                stroke="#e5e7eb"
                vertical={false}
              />
              <XAxis
                dataKey="name"
                tickLine={false}
                axisLine={{ stroke: "#e5e7eb" }}
              />
              <YAxis
                tickLine={false}
                axisLine={{ stroke: "#e5e7eb" }}
                allowDecimals={false}
                label={{
                  value: "Exercises",
                  angle: -90,
                  position: "insideLeft",
                  style: {
                    textAnchor: "middle",
                    fontSize: 12,
                    fill: "#6b7280",
                  },
                }}
              />
              <Tooltip content={<CustomTooltip />} />
              <Bar dataKey="value" name="Exercises">
                {weeklyData.map((entry, index) => {
                  let color = "#3b82f6";
                  if (entry.value > 0) {
                    if (entry.good >= entry.medium && entry.good >= entry.bad) {
                      color = "#22c55e";
                    } else if (
                      entry.medium >= entry.good &&
                      entry.medium >= entry.bad
                    ) {
                      color = "#eab308";
                    } else if (
                      entry.bad >= entry.good &&
                      entry.bad >= entry.medium
                    ) {
                      color = "#ef4444";
                    }
                  }
                  return <Cell key={`cell-${index}`} fill={color} />;
                })}
              </Bar>
            </BarChart>
          </ResponsiveContainer>
        </div>
      </CardContent>
    </Card>
  );
}
