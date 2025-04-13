import "~/styles/globals.css";

import { GeistSans } from "geist/font/sans";
import { type Metadata } from "next";

import { TRPCReactProvider } from "~/trpc/react";
import { CameraProvider } from "./_components/CameraContext";
import { NetworkProvider } from "./_components/NetworkContext";
import { WebSocketProvider } from "./_components/WebSocketProvider";
import NetworkStatus from "./_components/NetworkStatus";
import RealtimeNotification from "./_components/RealtimeNotification";
import { Toaster } from "~/components/ui/sonner";
import { SessionProvider } from "~/components/providers/session-provider";
import Navbar from "./_components/Navbar";

export const metadata: Metadata = {
  title: "Optima",
  description: "Track your form",
  icons: [{ rel: "icon", url: "/favicon.ico" }],
  viewport: {
    width: "device-width",
    initialScale: 1,
    maximumScale: 1,
    userScalable: false,
  },
};

export default function RootLayout({
  children,
}: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="en" className={`${GeistSans.variable}`}>
      <body className="min-h-screen bg-black text-white">
        <SessionProvider>
          <TRPCReactProvider>
            <NetworkProvider>
              <WebSocketProvider>
                <CameraProvider>
                  <Navbar />
                  <main>{children}</main>
                  <NetworkStatus />
                  <RealtimeNotification />
                </CameraProvider>
              </WebSocketProvider>
            </NetworkProvider>
          </TRPCReactProvider>
        </SessionProvider>
        <Toaster />
      </body>
    </html>
  );
}
