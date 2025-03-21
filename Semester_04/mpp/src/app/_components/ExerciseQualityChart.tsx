"use client";

import { Exercise } from "./CameraContext";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";

interface ExerciseQualityChartProps {
  exercises: Exercise[];
}

export default function ExerciseQualityChart({
  exercises,
}: ExerciseQualityChartProps) {
  if (!exercises.length) {
    return null;
  }

  const formCounts = {
    bad: exercises.filter((ex) => ex.form === "bad").length,
    medium: exercises.filter((ex) => ex.form === "medium").length,
    good: exercises.filter((ex) => ex.form === "good").length,
  };

  const totalExercises = exercises.length;
  const formPercentages = {
    bad: Math.round((formCounts.bad / totalExercises) * 100),
    medium: Math.round((formCounts.medium / totalExercises) * 100),
    good: Math.round((formCounts.good / totalExercises) * 100),
  };

  const segments = [
    { type: "bad", percentage: formPercentages.bad, color: "#ef4444" },
    { type: "medium", percentage: formPercentages.medium, color: "#eab308" },
    { type: "good", percentage: formPercentages.good, color: "#22c55e" },
  ].filter((segment) => segment.percentage > 0);

  const size = 160;
  const radius = size / 2;
  const center = size / 2;

  let currentAngle = 0;
  const segmentPaths = segments.map((segment) => {
    const angle = (segment.percentage / 100) * 360;
    const endAngle = currentAngle + angle;

    const startX = center + radius * Math.cos((Math.PI * currentAngle) / 180);
    const startY = center + radius * Math.sin((Math.PI * currentAngle) / 180);
    const endX = center + radius * Math.cos((Math.PI * endAngle) / 180);
    const endY = center + radius * Math.sin((Math.PI * endAngle) / 180);

    const largeArcFlag = angle > 180 ? 1 : 0;

    const path = [
      `M ${center} ${center}`,
      `L ${startX} ${startY}`,
      `A ${radius} ${radius} 0 ${largeArcFlag} 1 ${endX} ${endY}`,
      "Z",
    ].join(" ");

    const result = {
      path,
      color: segment.color,
      type: segment.type,
      percentage: segment.percentage,
      startAngle: currentAngle,
      endAngle,
    };

    currentAngle = endAngle;
    return result;
  });

  return (
    <Card className="border-border border shadow-sm">
      <CardHeader className="pb-2">
        <CardTitle className="text-lg font-medium">
          Form Quality Distribution
        </CardTitle>
        <CardDescription>Breakdown of exercise form quality</CardDescription>
      </CardHeader>
      <CardContent>
        <div className="flex flex-col items-center justify-center gap-6 sm:flex-row">
          <div className="relative h-40 w-40">
            <svg
              viewBox={`0 0 ${size} ${size}`}
              className="h-full w-full drop-shadow-sm"
            >
              {segmentPaths.map((segment, i) => (
                <path
                  key={i}
                  d={segment.path}
                  fill={segment.color}
                  stroke="white"
                  strokeWidth="1"
                >
                  <title>
                    {segment.type}: {segment.percentage}%
                  </title>
                </path>
              ))}

              <circle
                cx={center}
                cy={center}
                r={radius * 0.6}
                fill="white"
                className="drop-shadow-sm"
              />

              <text
                x={center}
                y={center}
                textAnchor="middle"
                dominantBaseline="middle"
                className="text-lg font-semibold"
              >
                {totalExercises}
              </text>
              <text
                x={center}
                y={center + 15}
                textAnchor="middle"
                dominantBaseline="middle"
                className="text-xs"
                fill="gray"
              >
                Total
              </text>
            </svg>
          </div>

          <div className="flex flex-col gap-3">
            {segments.map((segment, i) => (
              <div key={i} className="flex items-center gap-3">
                <div
                  className="h-4 w-4 rounded-sm shadow-sm"
                  style={{ backgroundColor: segment.color }}
                />
                <span className="font-medium capitalize">{segment.type}</span>
                <span className="text-sm text-gray-500">
                  {formCounts[segment.type as keyof typeof formCounts]} (
                  {segment.percentage}%)
                </span>
              </div>
            ))}
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
