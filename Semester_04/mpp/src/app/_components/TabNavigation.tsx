"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { Camera, Image, Settings } from "lucide-react";

const tabs = [
  {
    name: "Camera",
    href: "/",
    icon: Camera,
  },
  {
    name: "Gallery",
    href: "/gallery",
    icon: Image,
  },
  {
    name: "Settings",
    href: "/settings",
    icon: Settings,
  },
];

export default function TabNavigation() {
  const pathname = usePathname();

  return (
    <div className="fixed bottom-0 left-0 right-0 border-t border-zinc-800 bg-zinc-900">
      <nav className="flex h-16 items-center justify-around">
        {tabs.map((tab) => {
          const isActive = pathname === tab.href;
          return (
            <Link
              key={tab.name}
              href={tab.href}
              className={`flex h-full w-full flex-col items-center justify-center transition-colors ${
                isActive ? "text-white" : "text-zinc-500 hover:text-zinc-300"
              }`}
            >
              <tab.icon className="mb-1 h-6 w-6" />
              <span className="text-xs">{tab.name}</span>
            </Link>
          );
        })}
      </nav>
    </div>
  );
}
