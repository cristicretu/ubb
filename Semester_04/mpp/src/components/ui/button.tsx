"use client";

import * as React from "react";
import { cn } from "~/lib/utils";

export interface ButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  variant?:
    | "default"
    | "destructive"
    | "outline"
    | "secondary"
    | "ghost"
    | "link";
  size?: "default" | "sm" | "lg" | "icon";
  asChild?: boolean;
}

const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  (
    {
      className,
      variant = "default",
      size = "default",
      asChild = false,
      children,
      ...props
    },
    ref,
  ) => {
    const variantStyles = {
      default: "bg-blue-600 text-white hover:bg-blue-700",
      destructive: "bg-red-600 text-white hover:bg-red-700",
      outline: "border border-zinc-700 bg-transparent hover:bg-zinc-800",
      secondary: "bg-zinc-700 text-white hover:bg-zinc-600",
      ghost: "bg-transparent hover:bg-zinc-800",
      link: "text-blue-500 underline-offset-4 hover:underline bg-transparent",
    };

    const sizeStyles = {
      default: "h-10 px-4 py-2",
      sm: "h-9 rounded-md px-3 text-xs",
      lg: "h-11 rounded-md px-8 text-base",
      icon: "h-10 w-10 p-0",
    };

    const buttonClasses = cn(
      "inline-flex items-center justify-center whitespace-nowrap rounded-md text-sm font-medium transition-colors focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-zinc-500 focus-visible:ring-offset-2 disabled:pointer-events-none disabled:opacity-50",
      variantStyles[variant],
      sizeStyles[size],
      className,
    );

    if (asChild && React.isValidElement(children)) {
      return React.cloneElement(children as React.ReactElement, {
        className: cn(
          buttonClasses,
          (children as React.ReactElement).props.className,
        ),
        ref,
        ...props,
      });
    }

    return (
      <button className={buttonClasses} ref={ref} {...props}>
        {children}
      </button>
    );
  },
);
Button.displayName = "Button";

export { Button };
