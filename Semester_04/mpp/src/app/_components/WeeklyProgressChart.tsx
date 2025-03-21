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

interface WeeklyProgressChartProps {
  exercises: Exercise[];
}

export default function WeeklyProgressChart({
  exercises,
}: WeeklyProgressChartProps) {
  const [weeklyData, setWeeklyData] = useState<{
    [key: string]: {
      count: number;
      totalDuration: number;
      good: number;
      medium: number;
      bad: number;
    };
  }>({});

  useEffect(() => {
    const calculateWeeklyData = () => {
      const data: {
        [key: string]: {
          count: number;
          totalDuration: number;
          good: number;
          medium: number;
          bad: number;
        };
      } = {};

      const now = new Date();
      const currentDay = now.getDay(); // 0 = Sunday, 6 = Saturday
      const startOfWeek = new Date(now);
      startOfWeek.setDate(now.getDate() - currentDay);
      startOfWeek.setHours(0, 0, 0, 0);

      const weekStarts = [];
      for (let i = 0; i < 4; i++) {
        const date = new Date(startOfWeek);
        date.setDate(startOfWeek.getDate() - 7 * i);
        weekStarts.push(date);
      }

      weekStarts.forEach((date) => {
        const weekLabel = `${date.getMonth() + 1}/${date.getDate()}`;
        data[weekLabel] = {
          count: 0,
          totalDuration: 0,
          good: 0,
          medium: 0,
          bad: 0,
        };
      });

      exercises.forEach((exercise) => {
        const exerciseDate = new Date(exercise.date);

        for (let i = 0; i < weekStarts.length; i++) {
          const weekStart = weekStarts[i];
          const nextWeekStart = i === 0 ? new Date() : weekStarts[i - 1];

          if (exerciseDate >= weekStart && exerciseDate < nextWeekStart) {
            const weekLabel = `${weekStart.getMonth() + 1}/${weekStart.getDate()}`;

            // Update week data
            data[weekLabel].count += 1;
            data[weekLabel].totalDuration += exercise.duration;
            data[weekLabel][exercise.form] += 1;

            break;
          }
        }
      });

      setWeeklyData(data);
    };

    calculateWeeklyData();
  }, [exercises]);

  const formatDuration = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins}:${secs.toString().padStart(2, "0")}`;
  };

  const maxBarHeight = 140;
  const barWidth = 20;
  const barGap = 4;
  const weeks = Object.keys(weeklyData).sort((a, b) => {
    const [aMonth, aDay] = a.split("/").map(Number);
    const [bMonth, bDay] = b.split("/").map(Number);

    if (aMonth !== bMonth) return aMonth - bMonth;
    return aDay - bDay;
  });

  const maxCount = Math.max(
    ...Object.values(weeklyData).map((data) => data.count),
    1,
  );
  const maxDuration = Math.max(
    ...Object.values(weeklyData).map((data) => data.totalDuration),
    1,
  );

  if (!exercises.length) {
    return null;
  }

  return (
    <Card className="border-border border shadow-sm">
      <CardHeader className="pb-2">
        <CardTitle className="text-lg font-medium">Weekly Progress</CardTitle>
        <CardDescription>Exercise frequency and time spent</CardDescription>
      </CardHeader>
      <CardContent>
        <div className="pt-4">
          {/* Chart Header */}
          <div className="mb-4 flex justify-between">
            <div className="flex items-center gap-2">
              <div className="h-3 w-3 rounded-sm bg-blue-500"></div>
              <span className="text-xs text-gray-500">Exercise Count</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="h-3 w-3 rounded-sm bg-purple-500"></div>
              <span className="text-xs text-gray-500">Total Duration</span>
            </div>
          </div>

          <div className="relative h-[200px]">
            <div className="absolute left-0 top-0 flex h-full w-8 flex-col justify-between py-2 text-xs text-gray-500">
              <span>{maxCount}</span>
              <span>{Math.floor(maxCount / 2)}</span>
              <span>0</span>
            </div>

            <div className="absolute right-0 top-0 flex h-full w-10 flex-col justify-between py-2 text-xs text-gray-500">
              <span>{formatDuration(maxDuration)}</span>
              <span>{formatDuration(Math.floor(maxDuration / 2))}</span>
              <span>0:00</span>
            </div>

            <div className="absolute inset-y-0 left-10 right-12 flex items-end justify-around">
              {weeks.map((week) => {
                const data = weeklyData[week];
                const countHeight = (data.count / maxCount) * maxBarHeight;
                const durationHeight =
                  (data.totalDuration / maxDuration) * maxBarHeight;

                return (
                  <div key={week} className="flex flex-col items-center">
                    <div className="relative flex h-[140px] items-end gap-1">
                      <div
                        className="w-5 rounded-t bg-blue-500 opacity-90 transition-all hover:opacity-100"
                        style={{
                          height: `${countHeight}px`,
                        }}
                        title={`${data.count} exercises`}
                      />

                      <div
                        className="w-5 rounded-t bg-purple-500 opacity-90 transition-all hover:opacity-100"
                        style={{
                          height: `${durationHeight}px`,
                        }}
                        title={`${formatDuration(data.totalDuration)} total duration`}
                      />
                    </div>
                    <span className="mt-2 text-xs text-gray-500">{week}</span>
                  </div>
                );
              })}
            </div>

            <div className="absolute inset-y-0 left-10 right-12 flex flex-col justify-between">
              <div className="border-b border-gray-200" />
              <div className="border-b border-gray-200" />
              <div className="border-b border-gray-200" />
            </div>
          </div>

          <div className="mt-6">
            <h3 className="mb-2 text-sm font-medium">Form Quality by Week</h3>
            <div className="grid grid-cols-4 gap-2">
              {weeks.map((week) => {
                const data = weeklyData[week];
                const total = data.good + data.medium + data.bad || 1;

                return (
                  <div key={week} className="text-center">
                    <p className="text-xs font-medium text-gray-700">{week}</p>
                    <div className="mt-1 h-2 w-full overflow-hidden rounded-full bg-gray-200">
                      <div className="flex h-full">
                        {data.good > 0 && (
                          <div
                            className="h-full bg-green-500"
                            style={{ width: `${(data.good / total) * 100}%` }}
                          />
                        )}
                        {data.medium > 0 && (
                          <div
                            className="h-full bg-yellow-500"
                            style={{ width: `${(data.medium / total) * 100}%` }}
                          />
                        )}
                        {data.bad > 0 && (
                          <div
                            className="h-full bg-red-500"
                            style={{ width: `${(data.bad / total) * 100}%` }}
                          />
                        )}
                      </div>
                    </div>
                    <div className="mt-1 flex justify-between text-[10px] text-gray-500">
                      <span>{Math.round((data.good / total) * 100)}%</span>
                      <span>{Math.round((data.medium / total) * 100)}%</span>
                      <span>{Math.round((data.bad / total) * 100)}%</span>
                    </div>
                  </div>
                );
              })}
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  );
}
