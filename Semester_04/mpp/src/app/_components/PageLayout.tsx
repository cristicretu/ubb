"use client";

import TabNavigation from "./TabNavigation";

interface PageLayoutProps {
  children: React.ReactNode;
}

export default function PageLayout({ children }: PageLayoutProps) {
  return (
    <div
      className="flex h-screen flex-col bg-black"
      style={{ height: "100vh", overflowY: "auto" }}
    >
      <main className="flex-1 overflow-y-auto pb-16">{children}</main>
      <TabNavigation />
    </div>
  );
}
