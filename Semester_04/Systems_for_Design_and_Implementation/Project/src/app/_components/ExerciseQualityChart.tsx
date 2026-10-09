"use client";

import { Exercise } from "./CameraContext";
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from "~/components/ui/card";
import {
  PieChart,
  Pie,
  Cell,
  ResponsiveContainer,
  Legend,
  Tooltip,
} from "recharts";

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

  // Format data for Recharts
  const chartData = [
    { name: "Bad", value: formCounts.bad, percentage: formPercentages.bad },
    {
      name: "Medium",
      value: formCounts.medium,
      percentage: formPercentages.medium,
    },
    { name: "Good", value: formCounts.good, percentage: formPercentages.good },
  ].filter((item) => item.value > 0);

  const COLORS = ["#ef4444", "#eab308", "#22c55e"];

  const renderCustomizedLabel = ({
    cx,
    cy,
    midAngle,
    innerRadius,
    outerRadius,
    percent,
  }: any) => {
    const RADIAN = Math.PI / 180;
    const radius = innerRadius + (outerRadius - innerRadius) * 0.5;
    const x = cx + radius * Math.cos(-midAngle * RADIAN);
    const y = cy + radius * Math.sin(-midAngle * RADIAN);

    return percent > 0.05 ? (
      <text
        x={x}
        y={y}
        fill="white"
        textAnchor="middle"
        dominantBaseline="central"
        fontSize={12}
        fontWeight="bold"
      >
        {`${(percent * 100).toFixed(0)}%`}
      </text>
    ) : null;
  };

  const CustomTooltip = ({ active, payload }: any) => {
    if (active && payload && payload.length) {
      return (
        <div className="rounded border border-gray-200 bg-white p-2 text-sm shadow-sm">
          <p className="font-medium">{`${payload[0].name}: ${payload[0].value}`}</p>
          <p className="text-gray-500">{`${payload[0].payload.percentage}% of total`}</p>
        </div>
      );
    }
    return null;
  };

  return (
    <Card className="border-border border shadow-sm">
      <CardHeader className="pb-2">
        <CardTitle className="text-lg font-medium">
          Form Quality Distribution
        </CardTitle>
        <CardDescription>Breakdown of exercise form quality</CardDescription>
      </CardHeader>
      <CardContent>
        <div className="h-[250px] w-full">
          <ResponsiveContainer width="100%" height="100%">
            <PieChart>
              <Pie
                data={chartData}
                cx="50%"
                cy="50%"
                labelLine={false}
                outerRadius={80}
                fill="#8884d8"
                dataKey="value"
                label={renderCustomizedLabel}
              >
                {chartData.map((entry, index) => (
                  <Cell
                    key={`cell-${index}`}
                    fill={COLORS[index % COLORS.length]}
                  />
                ))}
              </Pie>
              <Tooltip content={<CustomTooltip />} />
              <Legend
                formatter={(value, entry, index) => {
                  const item = chartData[index];
                  return `${value} - ${item.value} (${item.percentage}%)`;
                }}
              />
            </PieChart>
          </ResponsiveContainer>
        </div>
      </CardContent>
    </Card>
  );
}
