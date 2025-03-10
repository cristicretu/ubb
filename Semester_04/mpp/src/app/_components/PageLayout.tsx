"use client";

import TabNavigation from "./TabNavigation";

interface PageLayoutProps {
  children: React.ReactNode;
}

export default function PageLayout({ children }: PageLayoutProps) {
  return (
    <div className="flex min-h-screen flex-col bg-black">
      <main className="flex-1 pb-16">{children}</main>
      <TabNavigation />
    </div>
  );
}
