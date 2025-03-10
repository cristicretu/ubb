import "~/styles/globals.css";

import { GeistSans } from "geist/font/sans";
import { type Metadata } from "next";

import { TRPCReactProvider } from "~/trpc/react";
import { CameraProvider } from "./_components/CameraContext";

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
      <body className="min-h-screen overflow-hidden bg-black text-white">
        <TRPCReactProvider>
          <CameraProvider>{children}</CameraProvider>
        </TRPCReactProvider>
      </body>
    </html>
  );
}
