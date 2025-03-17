import * as React from "react";
import { cn } from "~/lib/utils";

export interface BadgeProps extends React.HTMLAttributes<HTMLDivElement> {
  variant?:
    | "default"
    | "secondary"
    | "destructive"
    | "outline"
    | "good"
    | "medium"
    | "bad";
}

function Badge({ className, variant = "default", ...props }: BadgeProps) {
  const variantStyles = {
    default: "border-transparent bg-blue-600 text-white hover:bg-blue-700",
    secondary: "border-transparent bg-zinc-700 text-white hover:bg-zinc-600",
    destructive: "border-transparent bg-red-600 text-white hover:bg-red-700",
    outline: "border-zinc-700 text-zinc-100",
    good: "border-transparent bg-green-600 text-white",
    medium: "border-transparent bg-yellow-500 text-white",
    bad: "border-transparent bg-red-600 text-white",
  };

  return (
    <div
      className={cn(
        "inline-flex items-center rounded-full border px-2.5 py-0.5 text-xs font-semibold transition-colors focus:outline-none focus:ring-2 focus:ring-zinc-500 focus:ring-offset-2",
        variantStyles[variant],
        className,
      )}
      {...props}
    />
  );
}

export { Badge };
