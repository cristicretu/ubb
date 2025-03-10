import Camera from "./_components/Camera";
import PageLayout from "./_components/PageLayout";

export default function Home() {
  return (
    <PageLayout>
      <div className="flex h-[calc(100vh-4rem)] items-center justify-center px-4">
        <div className="mx-auto flex h-full w-full max-w-md items-center justify-center">
          <Camera />
        </div>
      </div>
    </PageLayout>
  );
}
